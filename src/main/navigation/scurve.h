#include <inttypes.h>

#include "common/vector.h"

// segment 0 is the initial segment and holds the vehicle's initial position and velocity
// segments 1 to 7 are the acceleration segments
// segments 8 to 14 are the speed change segments
// segment 15 is the constant velocity segment
// segment 16 to 22 is the deceleration segment
#define SEGMENTS_MAX 23 // maximum number of time segments

// segment types
typedef enum {
    CONSTANT_JERK,
    POSITIVE_JERK,
    NEGATIVE_JERK
} SegmentType_e;

typedef struct {
    fpVector3_t track;       // total change in position from origin to destination
    fpVector3_t delta_unit;  // reference direction vector for path 
    uint8_t num_segs;   // number of time segments being used
    float snap_max;     // maximum snap magnitude
    float jerk_max;     // maximum jerk magnitude
    float accel_max;    // maximum acceleration magnitude
    float vel_max;      // maximum velocity magnitude
    float time;         // time that defines position on the path
    float position_sq;  // position (squared) on the path at the last time step (used to detect finish)
    struct {
        float jerk_ref;     // jerk reference value for time segment (the jerk at the beginning, middle or end depending upon the segment type)
        SegmentType_e seg_type;   // segment type (jerk is constant, increasing or decreasing)
        float end_time;     // final time value for segment
        float end_accel;    // final acceleration value for segment
        float end_vel;      // final velocity value for segment
        float end_pos;      // final position value for segment
    } segment[SEGMENTS_MAX];
} scurve_t;

typedef struct {
    fpVector3_t _origin;                // origin offset (in NEU frame)
    fpVector3_t _destination;           // destination offset (in NEU frame)
    fpVector3_t _origin_vel;            // the target velocity vector in NEU frame at the origin of the spline segment
    fpVector3_t _destination_vel;       // the target velocity vector in NEU frame at the destination point of the spline segment
    fpVector3_t _hermite_solution[4];   // array describing path between origin and destination
    float       _time;                  // current spline time (between 0 and 1) between origin and destination
    float       _speed_xy;              // maximum horizontal speed
    float       _speed_up;              // maximum speed upwards
    float       _speed_down;            // maximum speed downwards
    float       _accel_xy;              // maximum horizontal acceleration
    float       _accel_z;               // maximum vertical acceleration
    float       _origin_speed_max;      // maximum speed at origin
    float       _destination_speed_max; // maximum speed at destination
    bool        _reached_destination;   // true once vehicle has reached destination
    bool        _zero_length;           // true if spline is zero length
} splineCurve_t;

void scurveInit(scurve_t *scurve);
void scurveSetDestinationSpeedMax(scurve_t *scurve, float speed);
float scurveSetOriginSpeeMax(scurve_t *scurve, float speed);
void scurveCalculateTrack(scurve_t *scurve, const fpVector3_t origin, const fpVector3_t destination, float speed_xy, float speed_up, float speed_down, float accel_xy, float accel_z, float snap_maximum, float jerk_maximum);
void scurveSetSpeedMax(scurve_t *scurve, float speed_xy, float speed_up, float speed_down);
bool scurveFinished(scurve_t *scurve);
bool scurveAdvanceTargetAlongTrack(scurve_t *scurve, scurve_t *prev_leg, scurve_t *next_leg, float wp_radius, float accel_corner, bool fast_waypoint, float dt, fpVector3_t *target_pos, fpVector3_t *target_vel, fpVector3_t *target_accel);

void splineCurveSetSpeedAccel(splineCurve_t *splineCurve, float speed_xy, float speed_up, float speed_down, float accel_xy, float accel_z);
void splineCurveAdvanceTargetAlongTrack(splineCurve_t *splineCurve, float dt, fpVector3_t *target_pos, fpVector3_t *target_vel);
void splineCurve_set_origin_and_destination(splineCurve_t *splineCurve, const fpVector3_t origin, const fpVector3_t destination, const fpVector3_t origin_vel, const fpVector3_t destination_vel);