#include "navigation/scurve.h"

#define SEG_INIT                0
#define SEG_ACCEL_MAX           4
#define SEG_TURN_IN             4
#define SEG_ACCEL_END           7
#define SEG_SPEED_CHANGE_END    14
#define SEG_CONST               15
#define SEG_TURN_OUT            15
#define SEG_DECEL_END           22

#define SPLINE_FACTOR           4.0f // defines shape of curves.  larger numbers result in longer spline curves, lower numbers take a direct path
#define TANGENTIAL_ACCEL_SCALER 0.5f // the proportion of the maximum accel that can be used for tangential acceleration (aka in the direction of travel along the track)
#define LATERAL_ACCEL_SCALER    0.5f // the proportion of the maximum accel that can be used for lateral acceleration (aka crosstrack acceleration)

// add single S-Curve segment
// populate the information for the segment specified in the path by the index variable.
// the index variable is incremented to reference the next segment in the array
static void scurve_add_segment(scurve_t *scurve, uint8_t *index, float end_time, SegmentType_e seg_type, float jerk_ref, float end_accel, float end_vel, float end_pos)
{
    scurve->segment[*index].end_time = end_time;
    scurve->segment[*index].seg_type = seg_type;
    scurve->segment[*index].jerk_ref = jerk_ref;
    scurve->segment[*index].end_accel = end_accel;
    scurve->segment[*index].end_vel = end_vel;
    scurve->segment[*index].end_pos = end_pos;
    index++;
}

// initialise and clear the path
void scurveInit(scurve_t *scurve)
{
    scurve->snap_max = 0.0f;
    scurve->jerk_max = 0.0f;
    scurve->accel_max = 0.0f;
    scurve->vel_max = 0.0f;
    scurve->time = 0.0f;
    scurve->num_segs = SEG_INIT;
    scurve_add_segment(scurve, &scurve->num_segs, 0.0f, CONSTANT_JERK, 0.0f, 0.0f, 0.0f, 0.0f);
    vectorZero(&scurve->track);
    vectorZero(&scurve->delta_unit);
    scurve->position_sq = 0.0f;
}

// return true if the curve is valid.  Used to identify and protect against code errors
static bool scurve_valid(scurve_t *scurve)
{
    // check number of segments
    if (scurve->num_segs != SEGMENTS_MAX) {
        return false;
    }

    for (uint8_t i = 0; i < scurve->num_segs; i++) {
        // jerk_ref should be finite (i.e. not NaN or infinity)
        // time, accel, vel and pos should finite and not negative
        if (!isfinite(scurve->segment[i].jerk_ref) ||
            !isfinite(scurve->segment[i].end_time) ||
            !isfinite(scurve->segment[i].end_accel) ||
            !isfinite(scurve->segment[i].end_vel) || scurve->segment[i].end_vel < 0.0f ||
            !isfinite(scurve->segment[i].end_pos)) {
            return false;
        }

        // time and pos should be increasing
        if (i >= 1) {
            if (scurve->segment[i].end_time - scurve->segment[i-1].end_time < 0.0f || scurve->segment[i].end_pos - scurve->segment[i-1].end_pos < 0.0f) {
                return false;
            }
        }
    }

    // last segment should have zero acceleration
    if (scurve->segment[scurve->num_segs - 1].end_accel != 0.0f) {
        return false;
    }

    // if we get this far then the curve must be valid
    return true;
}

// kinematic_limit calculates the maximum acceleration or velocity in a given direction.
// based on horizontal and vertical limits.
static float kinematic_limit(fpVector3_t direction, float max_xy, float max_z_pos, float max_z_neg)
{
    if ((sq(direction.x) + sq(direction.y + sq(direction.z)) == 0.0f) || max_xy == 0.0f || max_z_pos == 0.0f || max_z_neg == 0.0f) {
        return 0.0f;
    }

    max_xy = fabsf(max_xy);
    max_z_pos = fabsf(max_z_pos);
    max_z_neg = fabsf(max_z_neg);

    vectorNormalize(&direction, &direction);
    const float xy_length = calc_length_pythagorean_2D(direction.x, direction.y);

    if (xy_length == 0.0f) {
        return direction.z > 0.0f ? max_z_pos : max_z_neg;
    }

    if (direction.z == 0.0f) {
        return max_xy;
    }

    const float slope = direction.z / xy_length;

    if (slope > 0.0f) {
        if (fabsf(slope) < max_z_pos / max_xy) {
            return max_xy / xy_length;
        }
        return fabsf(max_z_pos / direction.z);
    }

    if (fabsf(slope) < max_z_neg / max_xy) {
        return max_xy / xy_length;
    }

    return fabsf(max_z_neg / direction.z);
}

// set speed and acceleration limits for the path
static void scurve_set_kinematic_limits(scurve_t *scurve, const fpVector3_t origin, const fpVector3_t destination,
                                  float speed_xy, float speed_up, float speed_down,
                                  float accel_xy, float accel_z)
{
    fpVector3_t direction = { .v = { destination.x - origin.x, destination.y - origin.y, destination.z - origin.z }};
    const float track_speed_max = kinematic_limit(direction, fabsf(speed_xy), fabsf(speed_up), fabsf(speed_down));
    const float track_accel_max = kinematic_limit(direction, fabsf(accel_xy), fabsf(accel_z), fabsf(accel_z));

    scurve->vel_max = track_speed_max;
    scurve->accel_max = track_accel_max;
}

// calculate the segment times for the trigonometric S-Curve path defined by:
// Sm - duration of the raised cosine jerk profile
// Jm - maximum value of the raised cosine jerk profile
// V0 - initial velocity magnitude
// Am - maximum constant acceleration
// Vm - maximum constant velocity
// L - Length of the path
// tj_out, t2_out, t4_out, t6_out are the segment durations needed to achieve the kinematic path specified by the input variables
static void scurve_calculate_path(float Sm, float Jm, float V0, float Am, float Vm, float L,float *Jm_out, float *tj_out,  float *t2_out, float *t4_out, float *t6_out)
{
    // init outputs
    *Jm_out = 0.0f;
    *tj_out = 0.0f;
    *t2_out = 0.0f;
    *t4_out = 0.0f;
    *t6_out = 0.0f;

    // check for invalid arguments
    if (Sm  <= 0.0f || Jm <= 0.0f || Am <= 0.0f || Vm <= 0.0f || L <= 0.0f) {
        return;
    }

    if (V0 >= Vm) {
        // no velocity change so all segments as zero length
        return;
    }

    float tj = Jm * M_PI / (2 * Sm);
    float At = MIN(MIN(Am, 
        (Vm - V0) / (2.0f * tj) ), 
        (L + 4.0f * V0 * tj) / (4.0f * sq(tj)) );

    if (fabsf(At) < Jm * tj) {
        if (V0 == 0.0f) {
            // we do not have a solution for non-zero initial velocity
            tj = MIN( MIN( MIN( tj,
                powf((L * M_PI) / (8.0 * Sm), 1.0/4.0) ), 
                powf((Vm * M_PI) / (4.0 * Sm), 1.0/3.0) ), 
                fast_fsqrtf((Am * M_PI) / (2.0 * Sm)) );
            Jm = 2.0 * Sm * tj / M_PI;
            Am = Jm * tj;
        } else {
            // When doing speed change we use fixed tj and adjust Jm for small changes
            Am = At;
            Jm = Am / tj;
        }
        if ((Vm <= V0 + 2.0f * Am * tj) || (L <= 4.0f * V0 * tj + 4.0f * Am * sq(tj))) {
            // solution = 0 - t6 t4 t2 = 0 0 0
            *t2_out = 0.0f;
            *t4_out = 0.0f;
            *t6_out = 0.0f;
        } else {
            // solution = 2 - t6 t4 t2 = 0 1 0
            *t2_out = 0.0f;
            *t4_out = MIN(-(V0 - Vm + Am * tj + (Am * Am) / Jm) / Am, MAX(((Am * Am) * (-3.0f / 2.0f) + fast_fsqrtf((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm), ((Am * Am) * (-3.0f / 2.0f) - fast_fsqrtf((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm)));
            *t4_out = MAX(*t4_out, 0.0);
            *t6_out = 0.0f;
        }
    } else {
        if ((Vm < V0 + Am * tj + (Am * Am) / Jm) || (L < 1.0f / (Jm * Jm) * (Am * Am * Am + Am * Jm * (V0 * 2.0f + Am * tj * 2.0f)) + V0 * tj * 2.0f + Am * (tj * tj))) {
            // solution = 5 - t6 t4 t2 = 1 0 1
            Am = MIN(MIN(Am, MAX(Jm * (tj + fast_fsqrtf((V0 * -4.0f + Vm * 4.0f + Jm * (tj * tj)) / Jm)) * (-1.0f / 2.0f), Jm * (tj - fast_fsqrtf((V0 * -4.0f + Vm * 4.0f + Jm * (tj * tj)) / Jm)) * (-1.0f / 2.0f))), Jm * tj * (-2.0f / 3.0f) + ((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f)) * 1.0f / powf(fast_fsqrtf(powf(- (Jm * Jm) * L * (1.0f / 2.0f) + (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) - Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) + (Jm * Jm) * V0 * tj, 2.0f) - powf((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f), 3.0f)) + (Jm * Jm) * L * (1.0f / 2.0f) - (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) + Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) - (Jm * Jm) * V0 * tj, 1.0f / 3.0f) + powf(fast_fsqrtf(powf(- (Jm * Jm) * L * (1.0f / 2.0f) + (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) - Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) + (Jm * Jm) * V0 * tj, 2.0f) - powf((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f), 3.0f)) + (Jm * Jm) * L * (1.0f / 2.0f) - (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) + Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) - (Jm * Jm) * V0 * tj, 1.0f / 3.0f));
            *t2_out = Am / Jm - tj;
            *t4_out = 0.0f;
            *t6_out = *t2_out;
        } else {
            // solution = 7 - t6 t4 t2 = 1 1 1
            *t2_out = Am / Jm - tj;
            *t4_out = MIN(-(V0 - Vm + Am * tj + (Am * Am) / Jm) / Am, MAX(((Am * Am) * (-3.0f / 2.0f) + fast_fsqrtf((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm), ((Am * Am) * (-3.0f / 2.0f) - fast_fsqrtf((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm)));
            *t4_out = MAX(*t4_out, 0.0);
            *t6_out = *t2_out;
        }
    }

    *tj_out = tj;
    *Jm_out = Jm;

    // check outputs and reset back to zero if necessary
    if (!isfinite(*Jm_out) || *Jm_out < 0.0f ||
        !isfinite(*tj_out) || *tj_out < 0.0f ||
        !isfinite(*t2_out) || *t2_out < 0.0f ||
        !isfinite(*t4_out) || *t4_out < 0.0f ||
        !isfinite(*t6_out) || *t6_out < 0.0f) {
        *Jm_out = 0.0f;
        *t2_out = 0.0f;
        *t4_out = 0.0f;
        *t6_out = 0.0f;
    }
}

// generate constant jerk time segment
// calculate the information needed to populate the constant jerk segment from the segment duration tj and jerk J0
// the index variable is the position of this segment in the path array and is incremented to reference the next segment in the array
static void scurve_add_segment_const_jerk(scurve_t *scurve, uint8_t *index, float tj, float J0)
{
    // if no time increase copy previous segment
    if (tj <= 0.0f) {
        scurve_add_segment(scurve, index, scurve->segment[*index - 1].end_time,
                    CONSTANT_JERK,
                    J0,
                    scurve->segment[*index - 1].end_accel,
                    scurve->segment[*index - 1].end_vel,
                    scurve->segment[*index - 1].end_pos);
        return;
    }

    const float J = J0;
    const float T = scurve->segment[*index - 1].end_time + tj;
    const float A = scurve->segment[*index - 1].end_accel + J0 * tj;
    const float V = scurve->segment[*index - 1].end_vel + scurve->segment[*index - 1].end_accel * tj + 0.5f * J0 * sq(tj);
    const float P = scurve->segment[*index - 1].end_pos + scurve->segment[*index - 1].end_vel * tj + 0.5f * scurve->segment[*index - 1].end_accel * sq(tj) + (1.0f / 6.0f) * J0 * powf(tj, 3.0f);
    scurve_add_segment(scurve, index, T, CONSTANT_JERK, J, A, V, P);
}

// generate increasing jerk magnitude time segment based on a raised cosine profile
// calculate the information needed to populate the increasing jerk magnitude segment from the segment duration tj and jerk magnitude Jm
// the index variable is the position of this segment in the path array and is incremented to reference the next segment in the array
static void scurve_add_segment_incr_jerk(scurve_t *scurve, uint8_t *index, float tj, float Jm)
{
    // if no time increase copy previous segment
    if (tj <= 0.0f) {
        scurve_add_segment(scurve, index, scurve->segment[*index - 1].end_time,
                    CONSTANT_JERK,
                    0.0,
                    scurve->segment[*index - 1].end_accel,
                    scurve->segment[*index - 1].end_vel,
                    scurve->segment[*index - 1].end_pos);
        return;
    }

    const float Beta = M_PI / tj;
    const float Alpha = Jm * 0.5f;
    const float AT = Alpha * tj;
    const float VT = Alpha * (sq(tj) * 0.5f - 2.0f / sq(Beta));
    const float PT = Alpha * ((-1.0f / sq(Beta)) * tj + (1.0f / 6.0f) * powf(tj, 3.0f));

    const float J = Jm;
    const float T = scurve->segment[*index - 1].end_time + tj;
    const float A = scurve->segment[*index - 1].end_accel + AT;
    const float V = scurve->segment[*index - 1].end_vel + scurve->segment[*index - 1].end_accel * tj + VT;
    const float P = scurve->segment[*index - 1].end_pos + scurve->segment[*index - 1].end_vel * tj + 0.5f * scurve->segment[*index - 1].end_accel * sq(tj) + PT;
    scurve_add_segment(scurve, index, T, POSITIVE_JERK, J, A, V, P);
}

// generate decreasing jerk magnitude time segment based on a raised cosine profile
// calculate the information needed to populate the decreasing jerk magnitude segment from the segment duration tj and jerk magnitude Jm
// the index variable is the position of this segment in the path and is incremented to reference the next segment in the array
static void scurve_add_segment_decr_jerk(scurve_t *scurve, uint8_t *index, float tj, float Jm)
{
    // if no time increase copy previous segment
    if (tj <= 0.0f) {
        scurve_add_segment(scurve, index, scurve->segment[*index - 1].end_time,
                    CONSTANT_JERK,
                    0.0,
                    scurve->segment[*index - 1].end_accel,
                    scurve->segment[*index - 1].end_vel,
                    scurve->segment[*index - 1].end_pos);
        return;
    }

    const float Beta = M_PI / tj;
    const float Alpha = Jm * 0.5f;
    const float AT = Alpha * tj;
    const float VT = Alpha * (sq(tj) * 0.5f - 2.0f / sq(Beta));
    const float PT = Alpha * ((-1.0f / sq(Beta)) * tj + (1.0f / 6.0f) * powf(tj, 3.0f));
    const float A2T = Jm * tj;
    const float V2T = Jm * sq(tj);
    const float P2T = Alpha * ((-1.0f / sq(Beta)) * 2.0f * tj + (4.0f / 3.0f) * powf(tj, 3.0f));

    const float J = Jm;
    const float T = scurve->segment[*index - 1].end_time + tj;
    const float A = (scurve->segment[*index - 1].end_accel - AT) + A2T;
    const float V = (scurve->segment[*index - 1].end_vel - VT) + (scurve->segment[*index - 1].end_accel - AT) * tj + V2T;
    const float P = (scurve->segment[*index - 1].end_pos - PT) + (scurve->segment[*index - 1].end_vel - VT) * tj + 0.5f * (scurve->segment[*index - 1].end_accel - AT) * sq(tj) + P2T;
    scurve_add_segment(scurve, index, T, NEGATIVE_JERK, J, A, V, P);
}

// generate three consecutive segments forming a jerk profile
// the index variable is the position within the path array that this jerk profile should be added
// the index is incremented to reference the next segment in the array after the jerk profile
static void scurve_add_segments_jerk(scurve_t *scurve, uint8_t *index, float tj, float Jm, float Tcj)
{
    scurve_add_segment_incr_jerk(scurve, index, tj, Jm);
    scurve_add_segment_const_jerk(scurve, index, Tcj, Jm);
    scurve_add_segment_decr_jerk(scurve, index, tj, Jm);
}

// generate the segments for a path of length L
// the path consists of 23 segments
// 1 initial segment
// 7 segments forming the acceleration S-Curve
// 7 segments forming the velocity change S-Curve
// 1 constant velocity S-Curve
// 7 segments forming the deceleration S-Curve
static void scurve_add_segments(scurve_t *scurve, float L)
{
    if (L == 0.0f) {
        return;
    }

    float Jm, tj, t2, t4, t6;
    scurve_calculate_path(scurve->snap_max, scurve->jerk_max, 0.0f, scurve->accel_max, scurve->vel_max, L * 0.5f, &Jm, &tj, &t2, &t4, &t6);

    scurve_add_segments_jerk(scurve, &scurve->num_segs, tj, Jm, t2);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, t4, 0.0f);
    scurve_add_segments_jerk(scurve, &scurve->num_segs, tj, -Jm, t6);

    // remove numerical errors
    scurve->segment[SEG_ACCEL_END].end_accel = 0.0f;

    // add empty speed adjust segments
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, 0.0f, 0.0f);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, 0.0f, 0.0f);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, 0.0f, 0.0f);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, 0.0f, 0.0f);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, 0.0f, 0.0f);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, 0.0f, 0.0f);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, 0.0f, 0.0f);

    const float t15 = MAX(0.0f, (L - 2.0f * scurve->segment[SEG_SPEED_CHANGE_END].end_pos) / scurve->segment[SEG_SPEED_CHANGE_END].end_vel);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, t15, 0.0f);

    scurve_add_segments_jerk(scurve, &scurve->num_segs, tj, -Jm, t6);
    scurve_add_segment_const_jerk(scurve, &scurve->num_segs, t4, 0.0f);
    scurve_add_segments_jerk(scurve, &scurve->num_segs, tj, Jm, t2);

    // remove numerical errors
    scurve->segment[SEG_DECEL_END].end_accel = 0.0f;
    scurve->segment[SEG_DECEL_END].end_vel = 0.0f;
}

// generate a trigonometric track in 3D space that moves over a straight line
// between two points defined by the origin and destination
void scurveCalculateTrack(scurve_t *scurve, const fpVector3_t origin, const fpVector3_t destination,
                             float speed_xy, float speed_up, float speed_down,
                             float accel_xy, float accel_z,
                             float snap_maximum, float jerk_maximum)
{
    scurveInit(scurve);

    // leave track as zero length if origin and destination are equal or if the new track length squared is zero
    const fpVector3_t track_temp = { .v = { destination.x - origin.x, destination.y - origin.y, destination.z - origin.z }};
    if ((track_temp.x == 0.0f && track_temp.y == 0.0f && track_temp.z == 0.0f) || (sq(track_temp.x) + sq(track_temp.y) + sq(track_temp.x) == 0.0f)) {
        return;
    }

    // set snap_max and jerk max
    scurve->snap_max = snap_maximum;
    scurve->jerk_max = jerk_maximum;

    // update speed and acceleration limits along path
    scurve_set_kinematic_limits(scurve, origin, destination,
                                speed_xy, speed_up, speed_down,
                                accel_xy, accel_z);

    // avoid divide-by zeros. Path will be left as a zero length path
    if (scurve->snap_max <= 0.0f || scurve->jerk_max <= 0.0f || scurve->accel_max <= 0.0f || scurve->vel_max <= 0.0f) {
        return;
    }

    scurve->track = track_temp;
    const float track_length = calc_length_pythagorean_3D(scurve->track.x, scurve->track.y, scurve->track.z);
    if (track_length == 0.0f) {
        // avoid possible divide by zero
        vectorZero(&scurve->delta_unit);
    } else {
        scurve->delta_unit.x = scurve->track.x / track_length;
        scurve->delta_unit.y = scurve->track.y / track_length;
        scurve->delta_unit.z = scurve->track.z / track_length;
        scurve_add_segments(scurve, track_length);
    }

    // catch calculation errors
    if (!scurve_valid(scurve)) {
        scurveInit(scurve);
    }
}

// set the maximum vehicle speed at the origin
// returns the expected speed at the origin which will always be equal or lower than speed
float scurveSetOriginSpeeMax(scurve_t *scurve, float speed)
{
    // if path is zero length then start speed must be zero
    if (scurve->num_segs != SEGMENTS_MAX) {
        return 0.0f;
    }

    // avoid re-calculating if unnecessary
    if (scurve->segment[SEG_INIT].end_vel == speed) {
        return speed;
    }

    const float Vm = scurve->segment[SEG_ACCEL_END].end_vel;
    const float track_length = calc_length_pythagorean_3D(scurve->track.x, scurve->track.y, scurve->track.z);
    speed = MIN(speed, Vm);

    float Jm, tj, t2, t4, t6;
    scurve_calculate_path(scurve->snap_max, scurve->jerk_max, speed, scurve->accel_max, Vm, track_length * 0.5f, &Jm, &tj, &t2, &t4, &t6);

    uint8_t seg = SEG_INIT;
    scurve_add_segment(scurve, &seg, 0.0f, CONSTANT_JERK, 0.0f, 0.0f, speed, 0.0f);
    scurve_add_segments_jerk(scurve, &seg, tj, Jm, t2);
    scurve_add_segment_const_jerk(scurve, &seg, t4, 0.0f);
    scurve_add_segments_jerk(scurve, &seg, tj, -Jm, t6);

    // remove numerical errors
    scurve->segment[SEG_ACCEL_END].end_accel = 0.0f;

    // offset acceleration segment if we can't fit it all into half the original length
    const float dPstart = MIN(0.0f, track_length * 0.5f - scurve->segment[SEG_ACCEL_END].end_pos);
    const float dt =  dPstart / scurve->segment[SEG_ACCEL_END].end_vel;
    for (uint8_t i = SEG_INIT; i <= SEG_ACCEL_END; i++) {
        scurve->segment[i].end_time += dt;
        scurve->segment[i].end_pos += dPstart;
    }

    // add empty speed change segments and constant speed segment
    for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_SPEED_CHANGE_END; i++) {
        scurve->segment[i].seg_type = CONSTANT_JERK;
        scurve->segment[i].jerk_ref = 0.0f;
        scurve->segment[i].end_time = scurve->segment[SEG_ACCEL_END].end_time;
        scurve->segment[i].end_accel = 0.0f;
        scurve->segment[i].end_vel = scurve->segment[SEG_ACCEL_END].end_vel;
        scurve->segment[i].end_pos = scurve->segment[SEG_ACCEL_END].end_pos;
    }

    seg = SEG_CONST;
    scurve_add_segment_const_jerk(scurve, &seg, 0.0f, 0.0f);

    scurve_calculate_path(scurve->snap_max, scurve->jerk_max, 0.0f, scurve->accel_max, scurve->segment[SEG_CONST].end_vel, track_length * 0.5f, &Jm, &tj, &t2, &t4, &t6);

    scurve_add_segments_jerk(scurve, &seg, tj, -Jm, t6);
    scurve_add_segment_const_jerk(scurve, &seg, t4, 0.0f);
    scurve_add_segments_jerk(scurve, &seg, tj, Jm, t2);

    // remove numerical errors
    scurve->segment[SEG_DECEL_END].end_accel = 0.0f;
    scurve->segment[SEG_DECEL_END].end_vel = MAX(0.0f, scurve->segment[SEG_DECEL_END].end_vel);

    // add to constant velocity segment to end at the correct position
    const float dP = MAX(0.0f, track_length - scurve->segment[SEG_DECEL_END].end_pos);
    const float t15 =  dP / scurve->segment[SEG_CONST].end_vel;

    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        scurve->segment[i].end_time += t15;
        scurve->segment[i].end_pos += dP;
    }

    // catch calculation errors
    if (!scurve_valid(scurve)) {
        scurveInit(scurve);
        return 0.0f;
    }

    return speed;
}

// set the maximum vehicle speed at the destination
void scurveSetDestinationSpeedMax(scurve_t *scurve, float speed)
{
    // if path is zero length then all speeds must be zero
    if (scurve->num_segs != SEGMENTS_MAX) {
        return;
    }

    // avoid re-calculating if unnecessary
    if (scurve->segment[SEGMENTS_MAX-1].end_vel == speed) {
        return;
    }

    const float Vm = scurve->segment[SEG_CONST].end_vel;
    const float track_length = calc_length_pythagorean_3D(scurve->track.x, scurve->track.y, scurve->track.z);
    speed = MIN(speed, Vm);

    float Jm, tj, t2, t4, t6;
    scurve_calculate_path(scurve->snap_max, scurve->jerk_max, speed, scurve->accel_max, Vm, track_length * 0.5f, &Jm, &tj, &t2, &t4, &t6);

    uint8_t seg = SEG_CONST;
    scurve_add_segment_const_jerk(scurve, &seg, 0.0f, 0.0f);

    scurve_add_segments_jerk(scurve, &seg, tj, -Jm, t6);
    scurve_add_segment_const_jerk(scurve, &seg, t4, 0.0f);
    scurve_add_segments_jerk(scurve, &seg, tj, Jm, t2);

    // remove numerical errors
    scurve->segment[SEG_DECEL_END].end_accel = 0.0f;
    scurve->segment[SEG_DECEL_END].end_vel = MAX(0.0f, scurve->segment[SEG_DECEL_END].end_vel);

    // add to constant velocity segment to end at the correct position
    const float dP = MAX(0.0f, track_length - scurve->segment[SEG_DECEL_END].end_pos);
    const float t15 =  dP / scurve->segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        scurve->segment[i].end_time += t15;
        scurve->segment[i].end_pos += dP;
    }

    // catch calculation errors
    if (!scurve_valid(scurve)) {
        scurveInit(scurve);
    }
}

// return time offset used to initiate the turn onto leg
static float scurve_time_turn_in(scurve_t *scurve)
{
    if (scurve->num_segs != SEGMENTS_MAX) {
        return 0.0f;
    }

    return scurve->segment[SEG_TURN_IN].end_time;
}

// return time offset used to initiate the turn from leg
static float scurve_time_turn_out(scurve_t *scurve)
{
    if (scurve->num_segs != SEGMENTS_MAX) {
        return 0.0f;
    }

    return scurve->segment[SEG_TURN_OUT].end_time;
}

// time at the end of the sequence
static float scurve_time_end(scurve_t *scurve)
{
    if (scurve->num_segs != SEGMENTS_MAX) {
        return 0.0f;
    }

    return scurve->segment[SEG_DECEL_END].end_time;
}

// time left before sequence will complete
static float scurve_get_time_remaining(scurve_t *scurve)
{
    if (scurve->num_segs != SEGMENTS_MAX) {
        return 0.0f;
    }
    return scurve->segment[SEG_DECEL_END].end_time - scurve->time;
}

// increment the internal time
static void scurve_advance_time(scurve_t *scurve, float dt)
{
    scurve->time = MIN(scurve->time+dt, scurve_time_end(scurve));
}

// time has reached the end of the sequence
bool scurveFinished(scurve_t *scurve)
{
    return ((scurve->time >= scurve_time_end(scurve)) || (scurve->position_sq >= (sq(scurve->track.x) + sq(scurve->track.y) + sq(scurve->track.z))));
}

// calculate the jerk, acceleration, velocity and position at time time_now when running the constant jerk time segment
static void scurve_calc_javp_for_segment_const_jerk(float time_now, float J0, float A0, float V0, float P0, float *Jt, float *At, float *Vt, float *Pt)
{
    *Jt = J0;
    *At = A0 + J0 * time_now;
    *Vt = V0 + A0 * time_now + 0.5f * J0 * (time_now * time_now);
    *Pt = P0 + V0 * time_now + 0.5f * A0 * (time_now * time_now) + (1.0f / 6.0f) * J0 * (time_now * time_now * time_now);
}

// Calculate the jerk, acceleration, velocity and position at time time_now when running the increasing jerk magnitude time segment based on a raised cosine profile
static void scurve_calc_javp_for_segment_incr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float *Jt, float *At, float *Vt, float *Pt)
{
    if (tj <= 0.0f) {
        *Jt = 0.0;
        *At = A0;
        *Vt = V0;
        *Pt = P0;
        return;
    }

    const float Alpha = Jm * 0.5f;
    const float Beta = M_PI / tj;

    *Jt = Alpha * (1.0f - cosf(Beta * time_now));
    *At = A0 + Alpha * time_now - (Alpha / Beta) * sinf(Beta * time_now);
    *Vt = V0 + A0 * time_now + (Alpha * 0.5f) * (time_now * time_now) + (Alpha / (Beta * Beta)) * cosf(Beta * time_now) - Alpha / (Beta * Beta);
    *Pt = P0 + V0 * time_now + 0.5f * A0 * (time_now * time_now) + (-Alpha / (Beta * Beta)) * time_now + Alpha * (time_now * time_now * time_now) / 6.0f + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * time_now);
}

// Calculate the jerk, acceleration, velocity and position at time time_now when running the decreasing jerk magnitude time segment based on a raised cosine profile
static void scurve_calc_javp_for_segment_decr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float *Jt, float *At, float *Vt, float *Pt)
{
    if (tj <= 0.0f) {
        *Jt = 0.0;
        *At = A0;
        *Vt = V0;
        *Pt = P0;
        return;
    }

    const float Alpha = Jm * 0.5f;
    const float Beta = M_PI / tj;
    const float AT = Alpha * tj;
    const float VT = Alpha * ((tj * tj) * 0.5f - 2.0f / (Beta * Beta));
    const float PT = Alpha * ((-1.0f / (Beta * Beta)) * tj + (1.0f / 6.0f) * (tj * tj * tj));

    *Jt = Alpha * (1.0f - cosf(Beta * (time_now + tj)));
    *At = (A0 - AT) + Alpha * (time_now + tj) - (Alpha / Beta) * sinf(Beta * (time_now + tj));
    *Vt = (V0 - VT) + (A0 - AT) * time_now + 0.5f * Alpha * (time_now + tj) * (time_now + tj) + (Alpha / (Beta * Beta)) * cosf(Beta * (time_now + tj)) - Alpha / (Beta * Beta);
    *Pt = (P0 - PT) + (V0 - VT) * time_now + 0.5f * (A0 - AT) * (time_now * time_now) + (-Alpha / (Beta * Beta)) * (time_now + tj) + (Alpha / 6.0f) * (time_now + tj) * (time_now + tj) * (time_now + tj) + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * (time_now + tj));
}

// calculate the jerk, acceleration, velocity and position at the provided time
static void scurve_get_jerk_accel_vel_pos_at_time(scurve_t *scurve, float time_now, float *Jt_out, float *At_out, float *Vt_out, float *Pt_out)
{
    // start with zeros as function is void and we want to guarantee all outputs are initialised
    *Jt_out = 0.0f;
    *At_out = 0.0f;
    *Vt_out = 0.0f;
    *Pt_out = 0.0f;

    if (scurve->num_segs != SEGMENTS_MAX) {
        return;
    }

    SegmentType_e Jtype;
    uint8_t pnt = scurve->num_segs;
    float Jm, tj, T0, A0, V0, P0;

    // find active segment at time_now
    for (uint8_t i = 0; i < scurve->num_segs; i++) {
        if (time_now < scurve->segment[scurve->num_segs - 1 - i].end_time) {
            pnt = scurve->num_segs - 1 - i;
        }
    }

    if (pnt == 0) {
        Jtype = CONSTANT_JERK;
        Jm = 0.0f;
        tj = 0.0f;
        T0 = scurve->segment[pnt].end_time;
        A0 = scurve->segment[pnt].end_accel;
        V0 = scurve->segment[pnt].end_vel;
        P0 = scurve->segment[pnt].end_pos;
    } else if (pnt == scurve->num_segs) {
        Jtype = CONSTANT_JERK;
        Jm = 0.0f;
        tj = 0.0f;
        T0 = scurve->segment[pnt - 1].end_time;
        A0 = scurve->segment[pnt - 1].end_accel;
        V0 = scurve->segment[pnt - 1].end_vel;
        P0 = scurve->segment[pnt - 1].end_pos;
    } else {
        Jtype = scurve->segment[pnt].seg_type;
        Jm = scurve->segment[pnt].jerk_ref;
        tj = scurve->segment[pnt].end_time - scurve->segment[pnt - 1].end_time;
        T0 = scurve->segment[pnt - 1].end_time;
        A0 = scurve->segment[pnt - 1].end_accel;
        V0 = scurve->segment[pnt - 1].end_vel;
        P0 = scurve->segment[pnt - 1].end_pos;
    }

    switch (Jtype) {
    case CONSTANT_JERK:
        scurve_calc_javp_for_segment_const_jerk(time_now - T0, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;

    case POSITIVE_JERK:
        scurve_calc_javp_for_segment_incr_jerk(time_now - T0, tj, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;

    case NEGATIVE_JERK:
        scurve_calc_javp_for_segment_decr_jerk(time_now - T0, tj, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    }

    *Pt_out = MAX(0.0f, *Pt_out);
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the destination
static void scurve_move_to_pos_vel_accel(scurve_t *scurve, float dt, fpVector3_t *pos, fpVector3_t *vel, fpVector3_t *accel)
{
    scurve_advance_time(scurve, dt);
    float scurve_P1 = 0.0f;
    float scurve_V1, scurve_A1, scurve_J1;
    scurve_get_jerk_accel_vel_pos_at_time(scurve, scurve->time, &scurve_J1, &scurve_A1, &scurve_V1, &scurve_P1);
    pos->x += scurve->delta_unit.x * scurve_P1;
    pos->y += scurve->delta_unit.y * scurve_P1;
    pos->z += scurve->delta_unit.z * scurve_P1;
    vel->x += scurve->delta_unit.x * scurve_V1;
    vel->y += scurve->delta_unit.y * scurve_V1;
    vel->z += scurve->delta_unit.z * scurve_V1;
    accel->x += scurve->delta_unit.x * scurve_A1;
    accel->y += scurve->delta_unit.y * scurve_A1;
    accel->z += scurve->delta_unit.z * scurve_A1;
    scurve->position_sq = sq(scurve_P1);
    pos->x -= scurve->track.x;
    pos->y -= scurve->track.y;
    pos->z -= scurve->track.z;
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the origin
static void scurve_move_from_pos_vel_accel(scurve_t *scurve, float dt, fpVector3_t *pos, fpVector3_t *vel, fpVector3_t *accel)
{
    scurve_advance_time(scurve, dt);
    float scurve_P1 = 0.0f;
    float scurve_V1, scurve_A1, scurve_J1;
    scurve_get_jerk_accel_vel_pos_at_time(scurve, scurve->time, &scurve_J1, &scurve_A1, &scurve_V1, &scurve_P1);
    pos->x += scurve->delta_unit.x * scurve_P1;
    pos->y += scurve->delta_unit.y * scurve_P1;
    pos->z += scurve->delta_unit.z * scurve_P1;
    vel->x += scurve->delta_unit.x * scurve_V1;
    vel->y += scurve->delta_unit.y * scurve_V1;
    vel->z += scurve->delta_unit.z * scurve_V1;
    accel->x += scurve->delta_unit.x * scurve_A1;
    accel->y += scurve->delta_unit.y * scurve_A1;
    accel->z += scurve->delta_unit.z * scurve_A1;
    scurve->position_sq = sq(scurve_P1);
}

// return the position, velocity and acceleration vectors relative to the origin at a specified time along the path
static void scurve_move_from_time_pos_vel_accel(scurve_t *scurve, float time_now, fpVector3_t *pos, fpVector3_t *vel, fpVector3_t *accel)
{
    float scurve_P1 = 0.0f;
    float scurve_V1 = 0.0f, scurve_A1 = 0.0f, scurve_J1 = 0.0f;
    scurve_get_jerk_accel_vel_pos_at_time(scurve, time_now, &scurve_J1, &scurve_A1, &scurve_V1, &scurve_P1);
    pos->x += scurve->delta_unit.x * scurve_P1;
    pos->y += scurve->delta_unit.y * scurve_P1;
    pos->z += scurve->delta_unit.z * scurve_P1;
    vel->x += scurve->delta_unit.x * scurve_V1;
    vel->y += scurve->delta_unit.y * scurve_V1;
    vel->z += scurve->delta_unit.z * scurve_V1;
    accel->x += scurve->delta_unit.x * scurve_A1;
    accel->y += scurve->delta_unit.y * scurve_A1;
    accel->z += scurve->delta_unit.z * scurve_A1;
}

// set maximum velocity and re-calculate the path using these limits
void scurveSetSpeedMax(scurve_t *scurve, float speed_xy, float speed_up, float speed_down)
{
    // return immediately if zero length path
    if (scurve->num_segs != SEGMENTS_MAX) {
        return;
    }

    // segment accelerations can not be changed after segment creation.
    const float track_speed_max = kinematic_limit(scurve->delta_unit, speed_xy, speed_up, fabsf(speed_down));

    if (scurve->vel_max == track_speed_max) {
        // new speed is equal to current speed maximum so no need to change anything
        return;
    }

    if (track_speed_max == 0.0f) {
        // new speed is zero which is not supported
        return;
    }

    scurve->vel_max = track_speed_max;

    if (scurve->time >= scurve->segment[SEG_CONST].end_time) {
        return;
    }

    // re-calculate the s-curve path based on update speeds

    const float Pend = scurve->segment[SEG_DECEL_END].end_pos;
    float Vend = MIN(scurve->vel_max, scurve->segment[SEG_DECEL_END].end_vel);

    if (scurve->time == 0.0f) {
        // path has not started so we can recompute the path
        const float Vstart = MIN(scurve->vel_max, scurve->segment[SEG_INIT].end_vel);
        scurve->num_segs = SEG_INIT;
        scurve_add_segment(scurve, &scurve->num_segs, 0.0f, CONSTANT_JERK, 0.0f, 0.0f, 0.0f, 0.0f);
        scurve_add_segments(scurve, Pend);
        scurveSetOriginSpeeMax(scurve, Vstart);
        scurveSetDestinationSpeedMax(scurve, Vend);
        return;
    }

    if ((scurve->time >= scurve->segment[SEG_ACCEL_END].end_time) && (scurve->time <= scurve->segment[SEG_SPEED_CHANGE_END].end_time)) {
        // in the speed change phase
        // move speed change phase to acceleration phase to provide room for further speed adjustments

        // set initial segment to last acceleration segment
        scurve->segment[SEG_INIT].seg_type = CONSTANT_JERK;
        scurve->segment[SEG_INIT].jerk_ref = 0.0f;
        scurve->segment[SEG_INIT].end_time = scurve->segment[SEG_ACCEL_END].end_time;
        scurve->segment[SEG_INIT].end_accel = scurve->segment[SEG_ACCEL_END].end_accel;
        scurve->segment[SEG_INIT].end_vel = scurve->segment[SEG_ACCEL_END].end_vel;
        scurve->segment[SEG_INIT].end_pos = scurve->segment[SEG_ACCEL_END].end_pos;

        // move speed change segments to acceleration segments
        for (uint8_t i = SEG_INIT+1; i <= SEG_ACCEL_END; i++) {
            scurve->segment[i] = scurve->segment[i+7];
        }

        // set change segments to last acceleration speed
        for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_SPEED_CHANGE_END; i++) {
            scurve->segment[i].seg_type = CONSTANT_JERK;
            scurve->segment[i].jerk_ref = 0.0f;
            scurve->segment[i].end_time = scurve->segment[SEG_ACCEL_END].end_time;
            scurve->segment[i].end_accel = 0.0f;
            scurve->segment[i].end_vel = scurve->segment[SEG_ACCEL_END].end_vel;
            scurve->segment[i].end_pos = scurve->segment[SEG_ACCEL_END].end_pos;
        }

    } else if ((scurve->time > scurve->segment[SEG_SPEED_CHANGE_END].end_time) && (scurve->time <= scurve->segment[SEG_CONST].end_time)) {
        // in the constant speed phase
        // overwrite the acceleration and speed change phases with the current position and velocity

        // set initial segment to last acceleration segment
        scurve->segment[SEG_INIT].seg_type = CONSTANT_JERK;
        scurve->segment[SEG_INIT].jerk_ref = 0.0f;
        scurve->segment[SEG_INIT].end_time = scurve->segment[SEG_SPEED_CHANGE_END].end_time;
        scurve->segment[SEG_INIT].end_accel = 0.0f;
        scurve->segment[SEG_INIT].end_vel = scurve->segment[SEG_SPEED_CHANGE_END].end_vel;
        scurve->segment[SEG_INIT].end_pos = scurve->segment[SEG_SPEED_CHANGE_END].end_pos;

        // set acceleration and change segments to current constant speed
        float Jt_out, At_out, Vt_out, Pt_out;

        scurve_get_jerk_accel_vel_pos_at_time(scurve, scurve->time, &Jt_out, &At_out, &Vt_out, &Pt_out);

        for (uint8_t i = SEG_INIT+1; i <= SEG_SPEED_CHANGE_END; i++) {
            scurve->segment[i].seg_type = CONSTANT_JERK;
            scurve->segment[i].jerk_ref = 0.0f;
            scurve->segment[i].end_time = scurve->time;
            scurve->segment[i].end_accel = 0.0f;
            scurve->segment[i].end_vel = Vt_out;
            scurve->segment[i].end_pos = Pt_out;
        }
    }

    // adjust the INIT and ACCEL segments for new speed
    if ((scurve->time <= scurve->segment[SEG_ACCEL_MAX].end_time) && scurve->segment[SEG_ACCEL_MAX].end_time - scurve->segment[SEG_ACCEL_MAX-1].end_time > 0.0f && (scurve->vel_max < scurve->segment[SEG_ACCEL_END].end_vel) && scurve->segment[SEG_ACCEL_MAX].end_accel > 0.0f) {
        // path has not finished constant positive acceleration segment
        // reduce velocity as close to target velocity as possible

        const float Vstart = scurve->segment[SEG_INIT].end_vel;

        // minimum velocity that can be obtained by shortening SEG_ACCEL_MAX
        const float Vmin = scurve->segment[SEG_ACCEL_END].end_vel - scurve->segment[SEG_ACCEL_MAX].end_accel * (scurve->segment[SEG_ACCEL_MAX].end_time - MAX(scurve->time, scurve->segment[SEG_ACCEL_MAX-1].end_time));

        float Jm, tj, t2, t4, t6;
        scurve_calculate_path(scurve->snap_max, scurve->jerk_max, Vstart, scurve->accel_max, MAX(Vmin, scurve->vel_max), Pend * 0.5f, &Jm, &tj, &t2, &t4, &t6);

        uint8_t seg = SEG_INIT+1;
        scurve_add_segments_jerk(scurve, &seg, tj, Jm, t2);
        scurve_add_segment_const_jerk(scurve, &seg, t4, 0.0f);
        scurve_add_segments_jerk(scurve, &seg, tj, -Jm, t6);

        // remove numerical errors
        scurve->segment[SEG_ACCEL_END].end_accel = 0.0f;

        // add empty speed adjust segments
        for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CONST; i++) {
            scurve->segment[i].seg_type = CONSTANT_JERK;
            scurve->segment[i].jerk_ref = 0.0f;
            scurve->segment[i].end_time = scurve->segment[SEG_ACCEL_END].end_time;
            scurve->segment[i].end_accel = 0.0f;
            scurve->segment[i].end_vel = scurve->segment[SEG_ACCEL_END].end_vel;
            scurve->segment[i].end_pos = scurve->segment[SEG_ACCEL_END].end_pos;
        }

        scurve_calculate_path(scurve->snap_max, scurve->jerk_max, 0.0f, scurve->accel_max, MAX(Vmin, scurve->vel_max), Pend * 0.5f, &Jm, &tj, &t2, &t4, &t6);

        seg = SEG_CONST + 1;
        scurve_add_segments_jerk(scurve, &seg, tj, -Jm, t6);
        scurve_add_segment_const_jerk(scurve, &seg, t4, 0.0f);
        scurve_add_segments_jerk(scurve, &seg, tj, Jm, t2);

        // remove numerical errors
        scurve->segment[SEG_DECEL_END].end_accel = 0.0f;
        scurve->segment[SEG_DECEL_END].end_vel = MAX(0.0f, scurve->segment[SEG_DECEL_END].end_vel);

        // add to constant velocity segment to end at the correct position
        const float dP = MAX(0.0f, Pend - scurve->segment[SEG_DECEL_END].end_pos);
        const float t15 =  dP / scurve->segment[SEG_CONST].end_vel;
        for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
            scurve->segment[i].end_time += t15;
            scurve->segment[i].end_pos += dP;
        }
    }

    // adjust the speed change segments (8 to 14) for new speed
    // start with empty speed adjust segments
    for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_SPEED_CHANGE_END; i++) {
        scurve->segment[i].seg_type = CONSTANT_JERK;
        scurve->segment[i].jerk_ref = 0.0f;
        scurve->segment[i].end_time = scurve->segment[SEG_ACCEL_END].end_time;
        scurve->segment[i].end_accel = 0.0f;
        scurve->segment[i].end_vel = scurve->segment[SEG_ACCEL_END].end_vel;
        scurve->segment[i].end_pos = scurve->segment[SEG_ACCEL_END].end_pos;
    }

    if (scurve->vel_max != scurve->segment[SEG_ACCEL_END].end_vel) {
        // add velocity adjustment
        // check there is enough time to make velocity change
        // we use the approximation that the time will be distance/max_vel and 8 jerk segments
        const float L = scurve->segment[SEG_CONST].end_pos - scurve->segment[SEG_ACCEL_END].end_pos;
        float Jm = 0;
        float tj = 0;
        float t2 = 0;
        float t4 = 0;
        float t6 = 0;
        float jerk_time = MIN(powf((fabsf(scurve->vel_max - scurve->segment[SEG_ACCEL_END].end_vel) * M_PI) / (4 * scurve->snap_max), 1 / 3), scurve->jerk_max * M_PI / (2 * scurve->snap_max));
        if ((scurve->vel_max < scurve->segment[SEG_ACCEL_END].end_vel) && (jerk_time * 12.0f < L / scurve->segment[SEG_ACCEL_END].end_vel)) {
            // we have a problem here with small segments.
            scurve_calculate_path(scurve->snap_max, scurve->jerk_max, scurve->vel_max, scurve->accel_max, scurve->segment[SEG_ACCEL_END].end_vel, L * 0.5f, &Jm, &tj, &t6, &t4, &t2);
            Jm = -Jm;

        } else if ((scurve->vel_max > scurve->segment[SEG_ACCEL_END].end_vel) && (L/(jerk_time*12.0f) > scurve->segment[SEG_ACCEL_END].end_vel)) {
            float Vm = MIN(scurve->vel_max, L / (jerk_time*12.0f));
            scurve_calculate_path(scurve->snap_max, scurve->jerk_max, scurve->segment[SEG_ACCEL_END].end_vel, scurve->accel_max, Vm, L * 0.5f, &Jm, &tj, &t2, &t4, &t6);
        }

        uint8_t seg = SEG_ACCEL_END + 1;
        if (Jm != 0.0f && t2 > 0.0f && t4 > 0.0f && t6 > 0.0f) {
            scurve_add_segments_jerk(scurve, &seg, tj, Jm, t2);
            scurve_add_segment_const_jerk(scurve, &seg, t4, 0.0f);
            scurve_add_segments_jerk(scurve, &seg, tj, -Jm, t6);

            // remove numerical errors
            scurve->segment[SEG_SPEED_CHANGE_END].end_accel = 0.0f;
        }
    }

    // add deceleration segments
    // earlier check should ensure that we should always have sufficient time to stop
    uint8_t seg = SEG_CONST;
    Vend = MIN(Vend, scurve->segment[SEG_SPEED_CHANGE_END].end_vel);
    scurve_add_segment_const_jerk(scurve, &seg, 0.0f, 0.0f);

    if (Vend < scurve->segment[SEG_SPEED_CHANGE_END].end_vel) {
        float Jm, tj, t2, t4, t6;
        scurve_calculate_path(scurve->snap_max, scurve->jerk_max, Vend, scurve->accel_max, scurve->segment[SEG_CONST].end_vel, Pend - scurve->segment[SEG_CONST].end_pos, &Jm, &tj, &t2, &t4, &t6);
        scurve_add_segments_jerk(scurve, &seg, tj, -Jm, t6);
        scurve_add_segment_const_jerk(scurve, &seg, t4, 0.0f);
        scurve_add_segments_jerk(scurve, &seg, tj, Jm, t2);
    } else {
        // No deceleration is required
        for (uint8_t i = SEG_CONST+1; i <= SEG_DECEL_END; i++) {
            scurve->segment[i].seg_type = CONSTANT_JERK;
            scurve->segment[i].jerk_ref = 0.0f;
            scurve->segment[i].end_time = scurve->segment[SEG_CONST].end_time;
            scurve->segment[i].end_accel = 0.0f;
            scurve->segment[i].end_vel = scurve->segment[SEG_CONST].end_vel;
            scurve->segment[i].end_pos = scurve->segment[SEG_CONST].end_pos;
        }
    }

    // remove numerical errors
    scurve->segment[SEG_DECEL_END].end_accel = 0.0f;
    scurve->segment[SEG_DECEL_END].end_vel = MAX(0.0f, scurve->segment[SEG_DECEL_END].end_vel);

    // add to constant velocity segment to end at the correct position
    const float dP = MAX(0.0f, Pend - scurve->segment[SEG_DECEL_END].end_pos);
    const float t15 =  dP / scurve->segment[SEG_CONST].end_vel;

    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        scurve->segment[i].end_time += t15;
        scurve->segment[i].end_pos += dP;
    }

    // catch calculation errors
    if (!scurve_valid(scurve)) {
        scurveInit(scurve);
    }
}

// move target location along path from origin to destination
// prev_leg and next_leg are the paths before and after this path
// wp_radius is max distance from the waypoint at the apex of the turn
// fast_waypoint should be true if vehicle will not stop at end of this leg
// dt is the time increment the vehicle will move along the path
// target_pos should be set to this segment's origin and it will be updated to the current position target
// target_vel and target_accel are updated with new targets
// returns true if vehicle has passed the apex of the corner
bool scurveAdvanceTargetAlongTrack(scurve_t *scurve, scurve_t *prev_leg, scurve_t *next_leg, float wp_radius, float accel_corner, bool fast_waypoint, float dt, fpVector3_t *target_pos, fpVector3_t *target_vel, fpVector3_t *target_accel)
{
    scurve_move_to_pos_vel_accel(prev_leg, dt, target_pos, target_vel, target_accel);
    scurve_move_from_pos_vel_accel(scurve, dt, target_pos, target_vel, target_accel);
    bool s_finished = scurveFinished(scurve);

    // check for change of leg on fast waypoint
    const float time_to_destination = scurve_get_time_remaining(scurve);
    if (fast_waypoint 
        && next_leg->time == 0.0f
        && (scurve->time >= scurve_time_turn_out(scurve) - scurve_time_turn_in(next_leg)) 
        && (scurve->position_sq >= 0.25 * (sq(scurve->track.x) + sq(scurve->track.y) + sq(scurve->track.z)))) {

        fpVector3_t turn_pos = { .v = { -scurve->track.x, -scurve->track.y, -scurve->track.z }};
        fpVector3_t turn_vel, turn_accel;
        scurve_move_from_time_pos_vel_accel(scurve, scurve->time + time_to_destination * 0.5f, &turn_pos, &turn_vel, &turn_accel);
        scurve_move_from_time_pos_vel_accel(next_leg, time_to_destination * 0.5f, &turn_pos, &turn_vel, &turn_accel);
        const float speed_min = MIN(scurve->vel_max, next_leg->vel_max);
        if ((scurve_get_time_remaining(scurve) < scurve_time_end(next_leg) * 0.5f) &&
            (calc_length_pythagorean_3D(turn_pos.x, turn_pos.y, turn_pos.z) < wp_radius) &&
            (calc_length_pythagorean_2D(turn_vel.x, turn_vel.y) < speed_min) &&
            (calc_length_pythagorean_2D(turn_accel.x, turn_accel.y) < accel_corner)) {
                scurve_move_from_pos_vel_accel(next_leg, dt, target_pos, target_vel, target_accel);
        }
    } else if (next_leg->time != 0.0f) {
        scurve_move_from_pos_vel_accel(next_leg, dt, target_pos, target_vel, target_accel);
        if (next_leg->time >= scurve_get_time_remaining(scurve)) {
            s_finished = true;
        }
    }

    return s_finished;
}

// set maximum speed and acceleration
void splineCurveSetSpeedAccel(splineCurve_t *splineCurve, float speed_xy, float speed_up, float speed_down, float accel_xy, float accel_z)
{
    splineCurve->_speed_xy = fabsf(speed_xy);
    splineCurve->_speed_up = fabsf(speed_up);
    splineCurve->_speed_down = fabsf(speed_down);
    splineCurve->_accel_xy = fabsf(accel_xy);
    splineCurve->_accel_z = fabsf(accel_z);
}

// recalculate hermite_solution grid
static void splineCurve_update_solution(splineCurve_t *splineCurve, const fpVector3_t origin, const fpVector3_t dest, const fpVector3_t origin_vel, const fpVector3_t dest_vel)
{
    splineCurve->_hermite_solution[0] = origin;
    splineCurve->_hermite_solution[1] = origin_vel;
    splineCurve->_hermite_solution[2].x = -origin.x * 3.0f -origin_vel.x * 2.0f + dest.x * 3.0f - dest_vel.x;
    splineCurve->_hermite_solution[2].y = -origin.y * 3.0f -origin_vel.y * 2.0f + dest.y * 3.0f - dest_vel.y;
    splineCurve->_hermite_solution[2].z = -origin.z * 3.0f -origin_vel.z * 2.0f + dest.z * 3.0f - dest_vel.z;
    splineCurve->_hermite_solution[3].x = origin.x * 2.0f + origin_vel.x - dest.x * 2.0f + dest_vel.x;
    splineCurve->_hermite_solution[3].y = origin.y * 2.0f + origin_vel.y - dest.y * 2.0f + dest_vel.y;
    splineCurve->_hermite_solution[3].z = origin.z * 2.0f + origin_vel.z - dest.z * 2.0f + dest_vel.z;
}

// calculate target position and velocity from given spline time
static void splineCurve_calc_target_pos_vel(splineCurve_t *splineCurve, float time, fpVector3_t *position, fpVector3_t *velocity, fpVector3_t *acceleration, fpVector3_t *jerk)
{
    const float time_sq = sq(time);
    const float time_cubed = time_sq * time;

    position->x = splineCurve->_hermite_solution[0].x + 
                  splineCurve->_hermite_solution[1].x * time + 
                  splineCurve->_hermite_solution[2].x * time_sq + 
                  splineCurve->_hermite_solution[3].x * time_cubed;

    position->y = splineCurve->_hermite_solution[0].y + 
                  splineCurve->_hermite_solution[1].y * time + 
                  splineCurve->_hermite_solution[2].y * time_sq + 
                  splineCurve->_hermite_solution[3].y * time_cubed;

    position->z = splineCurve->_hermite_solution[0].z + 
                  splineCurve->_hermite_solution[1].z * time + 
                  splineCurve->_hermite_solution[2].z * time_sq + 
                  splineCurve->_hermite_solution[3].z * time_cubed;

    velocity->x = splineCurve->_hermite_solution[1].x + 
                  splineCurve->_hermite_solution[2].x * 2.0f * time + 
                  splineCurve->_hermite_solution[3].x * 3.0f * time_sq;

    velocity->y = splineCurve->_hermite_solution[1].y + 
                  splineCurve->_hermite_solution[2].y * 2.0f * time + 
                  splineCurve->_hermite_solution[3].z * 3.0f * time_sq;

    velocity->z = splineCurve->_hermite_solution[1].z + 
                  splineCurve->_hermite_solution[2].z * 2.0f * time + 
                  splineCurve->_hermite_solution[3].z * 3.0f * time_sq;

    acceleration->x = splineCurve->_hermite_solution[2].x * 2.0f + 
                      splineCurve->_hermite_solution[3].x * 6.0f * time;

    acceleration->y = splineCurve->_hermite_solution[2].y * 2.0f + 
                      splineCurve->_hermite_solution[3].y * 6.0f * time;

    acceleration->z = splineCurve->_hermite_solution[2].z * 2.0f + 
                      splineCurve->_hermite_solution[3].z * 6.0f * time;

    jerk->x = splineCurve->_hermite_solution[3].x * 6.0f;
    jerk->y = splineCurve->_hermite_solution[3].y * 6.0f;
    jerk->z = splineCurve->_hermite_solution[3].z * 6.0f;
}

// calculate the spline delta time for a given delta distance
// returns the spline position and velocity and maximum speed and acceleration the vehicle can travel without exceeding acceleration limits
static void splineCurve_calc_dt_speed_max(splineCurve_t *splineCurve, float time, float distance_delta, float *spline_dt, fpVector3_t *target_pos, fpVector3_t *spline_vel_unit, float *speed_max, float *accel_max)
{
    // initialise outputs
    *spline_dt = 0.0f;
    vectorZero(spline_vel_unit);
    *speed_max = 0.0f;
    *accel_max = 0.0f;

    // calculate target position and velocity using spline calculator
    fpVector3_t spline_vel;
    fpVector3_t spline_accel;
    fpVector3_t spline_jerk;

    splineCurve_calc_target_pos_vel(splineCurve, time, target_pos, &spline_vel, &spline_accel, &spline_jerk);

    // vel, accel and jerk should never all be zero
    if ((spline_vel.x == 0.0f && spline_vel.y == 0.0f && spline_vel.z == 0.0f) && 
        (spline_accel.x == 0.0f && spline_accel.y == 0.0f && spline_accel.z == 0.0f) && 
        (spline_jerk.x == 0.0f && spline_jerk.y == 0.0f && spline_jerk.z == 0.0f)) {
        splineCurve->_reached_destination = true;
        return;
    }

    // aircraft velocity and acceleration along the spline will be defined based on the aircraft kinematic limits
    // aircraft velocity along the spline should be reduced to ensure normal accelerations do not exceed kinematic limits
    const float spline_vel_length = calc_length_pythagorean_3D(spline_vel.x, spline_vel.y, spline_vel.z);
    if (spline_vel_length == 0.0f) {
        // if spline velocity is zero then direction must be defined by acceleration or jerk
        if (sq(spline_accel.x) + sq(spline_accel.y) + sq(spline_accel.z) == 0.0f) {
            // if acceleration is zero then direction must be defined by jerk
            const float spline_jerk_len = calc_length_pythagorean_3D(spline_jerk.x, spline_jerk.y, spline_jerk.z);
            spline_vel_unit->x = spline_jerk.x / spline_jerk_len;
            spline_vel_unit->y = spline_jerk.y / spline_jerk_len;
            spline_vel_unit->z = spline_jerk.z / spline_jerk_len;
            *spline_dt = powf(6.0f * distance_delta / spline_jerk_len, 1.0f / 3.0f);
        } else {
            // all spline acceleration is in the direction of travel
            const float spline_accel_len = calc_length_pythagorean_3D(spline_accel.x, spline_accel.y, spline_accel.z);
            spline_vel_unit->x = spline_accel.x / spline_accel_len;
            spline_vel_unit->y = spline_accel.y / spline_accel_len;
            spline_vel_unit->z = spline_accel.z / spline_accel_len;
            *spline_dt = fast_fsqrtf(2.0f * distance_delta / spline_accel_len);
        }
    } else {
        spline_vel_unit->x = spline_vel.x / spline_vel_length;
        spline_vel_unit->y = spline_vel.y / spline_vel_length;
        spline_vel_unit->z = spline_vel.z / spline_vel_length;
        *spline_dt = distance_delta / spline_vel_length;
    }

    // calculate acceleration normal to the direction of travel
    const float spline_accel_tangent_length = spline_accel.x * spline_vel_unit->x + spline_accel.y * spline_vel_unit->y + spline_accel.z * spline_vel_unit->z;
    fpVector3_t spline_accel_norm = { .v = { spline_accel.x - (spline_vel_unit->x * spline_accel_tangent_length),
                                             spline_accel.y - (spline_vel_unit->y * spline_accel_tangent_length),
                                             spline_accel.z - (spline_vel_unit->z * spline_accel_tangent_length) }};
    const float spline_accel_norm_length = calc_length_pythagorean_3D(spline_accel_norm.x, spline_accel_norm.y, spline_accel_norm.z);

    // limit the maximum speed along the track to that which will achieve a cornering (aka lateral) acceleration of LATERAL_SPEED_SCALER * acceleration limit
    const float tangential_speed_max = kinematic_limit(*spline_vel_unit, splineCurve->_speed_xy, splineCurve->_speed_up, splineCurve->_speed_down);
    const float accel_norm_max = LATERAL_ACCEL_SCALER * kinematic_limit(spline_accel_norm, splineCurve->_accel_xy, splineCurve->_accel_z, splineCurve->_accel_z);

    // sanity check to avoid divide by zero
    if (tangential_speed_max == 0.0f) {
        splineCurve->_reached_destination = true;
        return;
    }

    if ((accel_norm_max > 0.0f) && spline_accel_norm_length > 0.0f && spline_vel_length > 0.0f &&
         ((spline_accel_norm_length/accel_norm_max) > sq(spline_vel_length/tangential_speed_max))) {
        *speed_max = spline_vel_length / fast_fsqrtf(spline_accel_norm_length/accel_norm_max);
    } else {
        *speed_max = tangential_speed_max;
    }

    // calculate accel max and sanity check
    *accel_max = TANGENTIAL_ACCEL_SCALER * kinematic_limit(*spline_vel_unit, splineCurve->_accel_xy, splineCurve->_accel_z, splineCurve->_accel_z);

    if (*accel_max == 0.0f) {
        splineCurve->_reached_destination = true;
        return;
    }

    const float dist = calc_length_pythagorean_3D(splineCurve->_destination.x - target_pos->x, splineCurve->_destination.y - target_pos->y, splineCurve->_destination.z - target_pos->z);
    *speed_max = MIN(*speed_max, fast_fsqrtf(2.0f * *accel_max * (dist + sq(splineCurve->_destination_speed_max) / (2.0f * *accel_max))));
}

// move target location along track from origin to destination
void splineCurveAdvanceTargetAlongTrack(splineCurve_t *splineCurve, float dt, fpVector3_t *target_pos, fpVector3_t *target_vel)
{
    // handle zero length track
    if (splineCurve->_zero_length) {
        *target_pos = splineCurve->_destination;
        vectorZero(target_vel);
        return;
    }

    // calculate target position and velocity using spline calculator
    float speed_cms = calc_length_pythagorean_3D(target_vel->x, target_vel->y, target_vel->z);
    const float distance_delta = speed_cms * dt;
    float spline_dt;
    fpVector3_t spline_vel_unit;
    float speed_max;
    float accel_max;

    splineCurve_calc_dt_speed_max(splineCurve, splineCurve->_time, distance_delta, &spline_dt, target_pos, &spline_vel_unit, &speed_max, &accel_max);

    speed_cms = constrainf(speed_max, speed_cms - accel_max * dt, speed_cms + accel_max * dt);
    target_vel->x = spline_vel_unit.x * speed_cms;
    target_vel->y = spline_vel_unit.y * speed_cms;
    target_vel->z = spline_vel_unit.z * speed_cms;

    splineCurve->_time += spline_dt;

    // we will reach the destination in the next step so set reached_destination flag
    if (splineCurve->_time >= 1.0f) {
        splineCurve->_time = 1.0f;
        splineCurve->_reached_destination = true;
    }
}

// set origin and destination using position vectors
void splineCurve_set_origin_and_destination(splineCurve_t *splineCurve, const fpVector3_t origin, const fpVector3_t destination, const fpVector3_t origin_vel, const fpVector3_t destination_vel)
{
    // store origin and destination locations
    splineCurve->_origin = origin;
    splineCurve->_destination = destination;

    // handle zero length track
   splineCurve-> _zero_length = (sq(destination.x - origin.x) + sq(destination.y - origin.y) + sq(destination.z - origin.z)) == 0.0f;

    if (splineCurve->_zero_length) {
        splineCurve->_time = 1.0f;
        vectorZero(&splineCurve->_origin_vel);
        vectorZero(&splineCurve->_destination_vel);
        splineCurve->_reached_destination = true;
        splineCurve->_origin_speed_max = 0.0f;
        splineCurve->_destination_speed_max = 0.0f;
        return;
    }

    splineCurve->_origin_vel = origin_vel;
    splineCurve->_destination_vel = destination_vel;
    splineCurve->_reached_destination = false;

    // reset time
    // Note: _time could include left-over from previous waypoint
    splineCurve->_time = 0.0f;

    // code below ensures we don't get too much overshoot when the next segment is short
    const float vel_len = calc_length_pythagorean_3D(splineCurve->_origin_vel.x, splineCurve->_origin_vel.y, splineCurve->_origin_vel.z) + calc_length_pythagorean_3D(splineCurve->_destination_vel.x, splineCurve->_destination_vel.y, splineCurve->_destination_vel.z);
    const float pos_len = calc_length_pythagorean_3D(splineCurve->_destination.x - splineCurve->_origin.x, splineCurve->_destination.y - splineCurve->_origin.y, splineCurve->_destination.z - splineCurve->_origin.z) * SPLINE_FACTOR;

    if (vel_len > pos_len) {
        // if total start+stop velocity is more than four times the position difference
        // use a scaled down start and stop velocity
        const float vel_scaling = pos_len / vel_len;
        // update spline calculator
        const fpVector3_t origin_velocity = { .v = { splineCurve->_origin_vel.x * vel_scaling, splineCurve->_origin_vel.y * vel_scaling, splineCurve->_origin_vel.z * vel_scaling }};
        const fpVector3_t destination_velocity = { .v = { splineCurve->_destination_vel.x * vel_scaling, splineCurve->_destination_vel.y * vel_scaling, splineCurve->_destination_vel.z * vel_scaling }};
        splineCurve_update_solution(splineCurve, splineCurve->_origin, splineCurve->_destination, origin_velocity, destination_velocity);
    } else {
        // update spline calculator
        splineCurve_update_solution(splineCurve, splineCurve->_origin, splineCurve->_destination, splineCurve->_origin_vel, splineCurve->_destination_vel);
    }

    fpVector3_t target_pos;
    fpVector3_t spline_vel_unit;
    float spline_dt;
    float accel_max;

    splineCurve_calc_dt_speed_max(splineCurve, 0.0f, 0.0f, &spline_dt, &target_pos, &spline_vel_unit, &splineCurve->_origin_speed_max, &accel_max);

    if (splineCurve->_destination_vel.x == 0.0f && splineCurve->_destination_vel.y == 0.0f && splineCurve->_destination_vel.z == 0.0f) {
        splineCurve->_destination_speed_max = 0.0f;
    } else {
        splineCurve_calc_dt_speed_max(splineCurve, 1.0f, 0.0f, &spline_dt, &target_pos, &spline_vel_unit, &splineCurve->_destination_speed_max, &accel_max);
    }
}