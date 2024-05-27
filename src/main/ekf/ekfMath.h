#include "common/quaternion.h"
#include "common/vector.h"

// wrap an angle defined in radians to -PI ~ PI
static inline float wrap_2PI(const float radian)
{
  float res = fmodf(radian, 2.0f * M_PIf);

  if (res < 0)
  {
    res += (2.0f * M_PIf);
  }

  return res;
}

// wrap an angle defined in radians to -PI ~ PI
static inline float wrap_PI(const float radian)
{
  float res = wrap_2PI(radian);

  if (res > M_PIf)
  {
    res -= (2.0f * M_PIf);
  }

  return res;
}

static inline fpQuaternion_t quaternionDivision(const fpQuaternion_t v, const fpQuaternion_t v2)
{
  fpQuaternion_t ret;

  const float quat0 = v.q0;
  const float quat1 = v.q1;
  const float quat2 = v.q2;
  const float quat3 = v.q3;

  const float rquat0 = v2.q0;
  const float rquat1 = v2.q1;
  const float rquat2 = v2.q2;
  const float rquat3 = v2.q3;

  ret.q0 = rquat0 * quat0 + rquat1 * quat1 + rquat2 * quat2 + rquat3 * quat3;
  ret.q1 = rquat0 * quat1 - rquat1 * quat0 - rquat2 * quat3 + rquat3 * quat2;
  ret.q2 = rquat0 * quat2 + rquat1 * quat3 - rquat2 * quat0 - rquat3 * quat1;
  ret.q3 = rquat0 * quat3 - rquat1 * quat2 + rquat2 * quat1 - rquat3 * quat0;

  return ret;
}

// Function to multiply two quaternions and store the result in the first quaternion
static inline void quaternion_multiply_assign(fpQuaternion_t *q1, const fpQuaternion_t q2)
{
  const float w1 = q1->q0;
  const float x1 = q1->q1;
  const float y1 = q1->q2;
  const float z1 = q1->q3;

  const float w2 = q2.q0;
  const float x2 = q2.q1;
  const float y2 = q2.q2;
  const float z2 = q2.q3;

  q1->q0 = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  q1->q1 = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  q1->q2 = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  q1->q3 = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

// create a quaternion from its axis-angle representation
static inline void quaternion_from_axis_angle(fpQuaternion_t *quat, fpVector3_t v)
{
  float theta = calc_length_pythagorean_3D(v.x, v.y, v.z);
  
  // axis must be a unit vector as there is no check for length
  if (theta == 0.0f)
  {
    quat->q0 = 1.0f;
    quat->q1 = quat->q2 = quat->q3 = 0.0f;
    return;
  }

  v.x /= theta;
  v.y /= theta;
  v.z /= theta;

  const float st2 = sinf(0.5f * theta);

  quat->q0 = cosf(0.5f * theta);
  quat->q1 = v.x * st2;
  quat->q2 = v.y * st2;
  quat->q3 = v.z * st2;
}

// populate the supplied rotation matrix equivalent from this quaternion
static inline void quaternionToRotationMatrix(fpQuaternion_t q, fpMat3_t *m)
{
  const float q2q2 = q.q2 * q.q2;
  const float q2q3 = q.q2 * q.q3;
  const float q1q1 = q.q1 * q.q1;
  const float q1q2 = q.q1 * q.q2;
  const float q1q3 = q.q1 * q.q3;
  const float q0q1 = q.q0 * q.q1;
  const float q0q2 = q.q0 * q.q2;
  const float q0q3 = q.q0 * q.q3;
  const float q3q3 = q.q3 * q.q3;

  m->m[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
  m->m[0][1] = 2.0f * (q1q2 - q0q3);
  m->m[0][2] = 2.0f * (q1q3 + q0q2);
  m->m[1][0] = 2.0f * (q1q2 + q0q3);
  m->m[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
  m->m[1][2] = 2.0f * (q2q3 - q0q1);
  m->m[2][0] = 2.0f * (q1q3 - q0q2);
  m->m[2][1] = 2.0f * (q2q3 + q0q1);
  m->m[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

// convert this quaternion to a rotation vector where the direction of the vector represents
// the axis of rotation and the length of the vector represents the angle of rotation
static inline void quaternionToAxisAngleV(fpQuaternion_t q, fpVector3_t *v)
{
  const float length = sqrtf(sq(q.q1) + sq(q.q2) + sq(q.q3));

  v->x = q.q1;
  v->y = q.q2;
  v->z = q.q3;

  if (length != 0.0f)
  {
    v->x /= length;
    v->y /= length;
    v->z /= length;
    v->x *= wrap_PI(2.0f * atan2f(length, q.q0));
    v->y *= wrap_PI(2.0f * atan2f(length, q.q0));
    v->z *= wrap_PI(2.0f * atan2f(length, q.q0));
  }
}

// create a quaternion from Euler angles
static inline void quaternionFromEuler(fpQuaternion_t *q, float roll, float pitch, float yaw)
{
  const float cr2 = cosf(roll * 0.5f);
  const float cp2 = cosf(pitch * 0.5f);
  const float cy2 = cosf(yaw * 0.5f);
  const float sr2 = sinf(roll * 0.5f);
  const float sp2 = sinf(pitch * 0.5f);
  const float sy2 = sinf(yaw * 0.5f);

  q->q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  q->q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  q->q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  q->q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
}

// create eulers from a quaternion
static inline void quaternionToEuler(fpQuaternion_t q, float *roll, float *pitch, float *yaw)
{
  *roll = atan2f(2.0f * (q.q0 * q.q1 + q.q2 * q.q3), 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2));
  *pitch = asinf(2.0f * (q.q0 * q.q2 - q.q3 * q.q1));
  *yaw = atan2f(2.0f * (q.q0 * q.q3 + q.q1 * q.q2), 1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3));
}

// matrix multiplication by a vector
static inline fpVector3_t multiplyMatrixByVector(fpMat3_t m, fpVector3_t v)
{
  fpVector3_t vRet;

  vRet.x = m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z;
  vRet.y = m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z;
  vRet.z = m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z;

  return vRet;
}

static inline void zeroMatrix(fpMat3_t *matrix)
{
  // A
  matrix->m[0][0] = 0.0f;
  matrix->m[0][1] = 0.0f;
  matrix->m[0][2] = 0.0f;

  // B
  matrix->m[1][0] = 0.0f;
  matrix->m[1][1] = 0.0f;
  matrix->m[1][2] = 0.0f;

  // C
  matrix->m[2][0] = 0.0f;
  matrix->m[2][1] = 0.0f;
  matrix->m[2][2] = 0.0f;
}

static inline void identityMatrix(fpMat3_t *matrix)
{
  // A
  matrix->m[0][0] = 1.0f;
  matrix->m[0][1] = 0.0f;
  matrix->m[0][2] = 0.0f;

  // B
  matrix->m[1][0] = 0.0f;
  matrix->m[1][1] = 1.0f;
  matrix->m[1][2] = 0.0f;

  // C
  matrix->m[2][0] = 0.0f;
  matrix->m[2][1] = 0.0f;
  matrix->m[2][2] = 1.0f;
}

static inline fpMat3_t matrixTransposed(const fpMat3_t m)
{
  const fpMat3_t result = {{{m.m[0][0], m.m[1][0], m.m[2][0]},
                            {m.m[0][1], m.m[1][1], m.m[2][1]},
                            {m.m[0][2], m.m[1][2], m.m[2][2]}}};

  return result;
}

static inline void matrixFromEuler(fpMat3_t *m, float roll, float pitch, float yaw)
{
  const float cp = cos(pitch);
  const float sp = sin(pitch);
  const float sr = sin(roll);
  const float cr = cos(roll);
  const float sy = sin(yaw);
  const float cy = cos(yaw);

  m->m[0][0] = cp * cy;
  m->m[0][1] = (sr * sp * cy) - (cr * sy);
  m->m[0][2] = (cr * sp * cy) + (sr * sy);
  m->m[1][0] = cp * sy;
  m->m[1][1] = (sr * sp * sy) + (cr * cy);
  m->m[1][2] = (cr * sp * sy) - (sr * cy);
  m->m[2][0] = -sp;
  m->m[2][1] = sr * cp;
  m->m[2][2] = cr * cp;
}
