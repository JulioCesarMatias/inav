#include "ekf/ekf.h"
#include "ekf/ekfCore.h"

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
// Assume that the calibration is performed to an accuracy of 0.5 deg/sec which will require averaging under static conditions
// WARNING - a non-blocking calibration method must be used
void coreResetGyroBias(void)
{
    vectorZero(&ekfStates.stateStruct.gyro_bias);
    zeroRows(P, 9, 11);
    zeroCols(P, 9, 11);

    P[9][9] = sq(RADIANS_TO_DEGREES(0.5f * dtIMUavg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
}

/*
   vehicle specific initial gyro bias uncertainty in deg/sec
 */
float InitialGyroBiasUncertainty(void)
{
    return 2.5f;
}
