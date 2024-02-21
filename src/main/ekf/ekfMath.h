
#include "common/quaternion.h"
#include "common/vector.h"

// wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
static inline float wrap_PI(float angle_in_radians)
{
  if (angle_in_radians > 10 * M_PIf || angle_in_radians < -10 * M_PIf)
  {
    // for very large numbers use modulus
    angle_in_radians = fmodf(angle_in_radians, 2 * M_PIf);
  }

  while (angle_in_radians > M_PIf)
  {
    angle_in_radians -= 2 * M_PIf;
  }

  while (angle_in_radians < -M_PIf)
  {
    angle_in_radians += 2 * M_PIf;
  }

  return angle_in_radians;
}

// Function to calculate the squared length of a vector
static inline float vector_squared_length(fpVector3_t vec)
{
  return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
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

// matrix multiplication by a vector
static inline fpVector3_t multiplyMatrixByVector(fpMat3_t m, fpVector3_t v)
{
  fpVector3_t vRet;

  vRet.x = m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z;
  vRet.y = m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z;
  vRet.z = m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z;

  return vRet;
}

// multiplication of transpose by a vector
static inline fpVector3_t multiplyMatrixTransposeByVector(fpMat3_t m, fpVector3_t v)
{
  fpVector3_t vRet;

  vRet.x = m.m[0][0] * v.x + m.m[1][0] * v.y + m.m[2][0] * v.z;
  vRet.y = m.m[0][1] * v.x + m.m[1][1] * v.y + m.m[2][1] * v.z;
  vRet.z = m.m[0][2] * v.x + m.m[1][2] * v.y + m.m[2][2] * v.z;

  return vRet;
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

/*
  fill the matrix from Euler angles in radians in 312 convention
*/
static inline fpMat3_t matrix_from_euler312(float roll, float pitch, float yaw)
{
  fpMat3_t m;

  const float c3 = cosf(pitch);
  const float s3 = sinf(pitch);
  const float s2 = sinf(roll);
  const float c2 = cosf(roll);
  const float s1 = sinf(yaw);
  const float c1 = cosf(yaw);

  m.m[0][0] = c1 * c3 - s1 * s2 * s3;
  m.m[1][1] = c1 * c2;
  m.m[2][2] = c3 * c2;
  m.m[0][1] = -c2 * s1;
  m.m[0][2] = s3 * c1 + c3 * s2 * s1;
  m.m[1][0] = c3 * s1 + s3 * s2 * c1;
  m.m[1][2] = s1 * s3 - s2 * c1 * c3;
  m.m[2][0] = -s3 * c2;
  m.m[2][1] = s2;

  return m;
}

static inline fpMat3_t matrixTransposed(const fpMat3_t m)
{
  fpMat3_t result = {{{m.m[0][0], m.m[1][0], m.m[2][0]},
                      {m.m[0][1], m.m[1][1], m.m[2][1]},
                      {m.m[0][2], m.m[1][2], m.m[2][2]}}};

  return result;
}

static inline void quaternionInitialise(fpQuaternion_t *quat)
{
  quat->q0 = 1.0f;
  quat->q1 = 0.0f;
  quat->q2 = 0.0f;
  quat->q3 = 0.0f;
}

// Helper function to check if a value is close to zero
static inline bool is_zero(float val)
{
  return fabsf(val) < 1e-6; // Adjust the tolerance level as needed
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

  ret.q0 = (rquat0 * quat0 + rquat1 * quat1 + rquat2 * quat2 + rquat3 * quat3);
  ret.q1 = (rquat0 * quat1 - rquat1 * quat0 - rquat2 * quat3 + rquat3 * quat2);
  ret.q2 = (rquat0 * quat2 + rquat1 * quat3 - rquat2 * quat0 - rquat3 * quat1);
  ret.q3 = (rquat0 * quat3 - rquat1 * quat2 + rquat2 * quat1 - rquat3 * quat0);

  return ret;
}

// create a quaternion from its axis-angle representation
static inline void quaternion_from_axis_angle_helper(fpQuaternion_t *quat, fpVector3_t axis, float theta)
{
  // axis must be a unit vector as there is no check for length
  if (is_zero(theta))
  {
    quat->q0 = 1.0f;
    quat->q1 = quat->q2 = quat->q3 = 0.0f;
    return;
  }
  const float st2 = sinf(0.5f * theta);

  quat->q0 = cosf(0.5f * theta);
  quat->q1 = axis.x * st2;
  quat->q2 = axis.y * st2;
  quat->q3 = axis.z * st2;
}

// create a quaternion from its axis-angle representation
static inline void quaternion_from_axis_angle(fpQuaternion_t *quat, fpVector3_t v)
{
  float theta = calc_length_pythagorean_3D(v.x, v.y, v.z);
  if (is_zero(theta))
  {
    quat->q0 = 1.0f;
    quat->q1 = quat->q2 = quat->q3 = 0.0f;
    return;
  }
  v.x /= theta;
  v.y /= theta;
  v.z /= theta;
  quaternion_from_axis_angle_helper(quat, v, theta);
}

// rotate by the provided axis angle
static inline void quaternion_rotate(fpQuaternion_t *quat, fpVector3_t v)
{
  fpQuaternion_t r;
  quaternion_from_axis_angle(&r, v);
  quat->q0 *= r.q0;
  quat->q1 *= r.q1; 
  quat->q2 *= r.q2; 
  quat->q3 *= r.q3;

  // Perform quaternion multiplication
  /*float q0 = quat->q0;
  float q1 = quat->q1;
  float q2 = quat->q2;
  float q3 = quat->q3;
  quat->q0 = q0 * r.q0 - q1 * r.q1 - q2 * r.q2 - q3 * r.q3;
  quat->q1 = q0 * r.q1 + q1 * r.q0 + q2 * r.q3 - q3 * r.q2;
  quat->q2 = q0 * r.q2 - q1 * r.q3 + q2 * r.q0 + q3 * r.q1;
  quat->q3 = q0 * r.q3 + q1 * r.q2 - q2 * r.q1 + q3 * r.q0;*/
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

// make this quaternion equivalent to the supplied matrix
// Thanks to Martin John Baker
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
static inline fpQuaternion_t quaternion_from_rotation_matrix(fpMat3_t m)
{
  const float m00 = m.m[0][0];
  const float m11 = m.m[1][1];
  const float m22 = m.m[2][2];
  const float m10 = m.m[1][0];
  const float m01 = m.m[0][1];
  const float m20 = m.m[2][0];
  const float m02 = m.m[0][2];
  const float m21 = m.m[2][1];
  const float m12 = m.m[1][2];
  fpQuaternion_t q;

  const float tr = m00 + m11 + m22;

  if (tr > 0)
  {
    const float S = sqrtf(tr + 1) * 2;
    q.q0 = 0.25f * S;
    q.q1 = (m21 - m12) / S;
    q.q2 = (m02 - m20) / S;
    q.q3 = (m10 - m01) / S;
  }
  else if ((m00 > m11) && (m00 > m22))
  {
    const float S = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
    q.q0 = (m21 - m12) / S;
    q.q1 = 0.25f * S;
    q.q2 = (m01 + m10) / S;
    q.q3 = (m02 + m20) / S;
  }
  else if (m11 > m22)
  {
    const float S = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
    q.q0 = (m02 - m20) / S;
    q.q1 = (m01 + m10) / S;
    q.q2 = 0.25f * S;
    q.q3 = (m12 + m21) / S;
  }
  else
  {
    const float S = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
    q.q0 = (m10 - m01) / S;
    q.q1 = (m02 + m20) / S;
    q.q2 = (m12 + m21) / S;
    q.q3 = 0.25f * S;
  }

  return q;
}

// create a quaternion from Euler angles
static inline void quaternionFromEuler(fpQuaternion_t *q, float roll, float pitch, float yaw)
{
  const float cr2 = cos_approx(roll * 0.5f);
  const float cp2 = cos_approx(pitch * 0.5f);
  const float cy2 = cos_approx(yaw * 0.5f);
  const float sr2 = sin_approx(roll * 0.5f);
  const float sp2 = sin_approx(pitch * 0.5f);
  const float sy2 = sin_approx(yaw * 0.5f);

  q->q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  q->q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  q->q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  q->q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
}

static inline void quaternionToEuler(fpQuaternion_t q, float *roll, float *pitch, float *yaw)
{
  *roll = atan2_approx(2.0f * (q.q0 * q.q1 + q.q2 * q.q3), 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2));
  *pitch = asin_approx(2.0f * (q.q0 * q.q2 - q.q3 * q.q1));
  *yaw = atan2_approx(2.0f * (q.q0 * q.q3 + q.q1 * q.q2), 1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3));
}

// convert this quaternion to a rotation vector where the direction of the vector represents
// the axis of rotation and the length of the vector represents the angle of rotation
static inline void quaternionToAxisAngleV(fpQuaternion_t q, fpVector3_t *v)
{
  const float length = sqrtf(sq(q.q1) + sq(q.q2) + sq(q.q3));
  v->x = q.q1;
  v->y = q.q2;
  v->z = q.q3;
  if (!is_zero(length))
  {
    v->x /= length;
    v->y /= length;
    v->z /= length;
    v->x *= wrap_PI(2.0f * atan2f(length, q.q0));
    v->y *= wrap_PI(2.0f * atan2f(length, q.q0));
    v->z *= wrap_PI(2.0f * atan2f(length, q.q0));
  }
}

// create eulers from a quaternion
static inline fpVector3_t quaternion_to_vector312(fpQuaternion_t q)
{
  fpMat3_t m;
  fpVector3_t v;

  quaternionToRotationMatrix(q, &m);

  /*
    calculate Euler angles (312 convention) for the matrix.
    See http://www.atacolorado.com/eulersequences.doc
    vector is returned in r, p, y order
  */
  v.x = asinf(m.m[2][1]);
  v.y = atan2f(-m.m[2][0], m.m[2][2]);
  v.z = atan2f(-m.m[0][1], m.m[1][1]);

  return v;
}

// create a quaternion from Euler angles applied in yaw, roll, pitch order
// instead of the normal yaw, pitch, roll order
static inline fpQuaternion_t quaternion_from_vector312(float roll, float pitch, float yaw)
{
  fpMat3_t m = matrix_from_euler312(roll, pitch, yaw);

  return quaternion_from_rotation_matrix(m);
}

// return the reverse rotation of this quaternion
static inline fpQuaternion_t quaternion_inverse(fpQuaternion_t q)
{
  const fpQuaternion_t qRet = {q.q0, -q.q1, -q.q2, -q.q3};

  return qRet;
}