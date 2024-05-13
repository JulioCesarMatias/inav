#include "ekf/ekfCore.h"
#include "ekf/ekfBuffer.h"
#include "ekf/ekfIntegrator.h"

imuIntegrator accel_integrator;
imuIntegratorConing gyro_integrator;

bool accIntegratorInit;
bool gyroIntegratorInit;

// Read the range finder and take new measurements if available and apply a median filter
void readRangeFinder(void)
{
    uint8_t midIndex;
    uint8_t maxIndex;
    uint8_t minIndex;

    rngOnGnd = 0.05f; // Distance (in cm) from the range finder to the ground

    // read range finder at 20Hz
    if ((imuSampleTime_ms - lastRngMeasTime_ms) > 50)
    {
        // reset the timer used to control the measurement rate
        lastRngMeasTime_ms = imuSampleTime_ms;

        // store samples and sample time into a ring buffer if valid
        if (rangefinderIsHealthy())
        {
            rngMeasIndex++;
            if (rngMeasIndex > 2)
            {
                rngMeasIndex = 0;
            }
            storedRngMeasTime_ms[rngMeasIndex] = imuSampleTime_ms - 25;
            storedRngMeas[rngMeasIndex] = CENTIMETERS_TO_METERS(rangefinderGetLatestRawAltitude());
        }

        // check for three fresh samples
        bool sampleFresh[3];
        for (uint8_t index = 0; index <= 2; index++)
        {
            sampleFresh[index] = (imuSampleTime_ms - storedRngMeasTime_ms[index]) < 500;
        }

        // find the median value if we have three fresh samples
        if (sampleFresh[0] && sampleFresh[1] && sampleFresh[2])
        {
            if (storedRngMeas[0] > storedRngMeas[1])
            {
                minIndex = 1;
                maxIndex = 0;
            }
            else
            {
                minIndex = 0;
                maxIndex = 1;
            }
            if (storedRngMeas[2] > storedRngMeas[maxIndex])
            {
                midIndex = maxIndex;
            }
            else if (storedRngMeas[2] < storedRngMeas[minIndex])
            {
                midIndex = minIndex;
            }
            else
            {
                midIndex = 2;
            }

            // don't allow time to go backwards
            if (storedRngMeasTime_ms[midIndex] > rangeDataNew.obs.time_ms)
            {
                rangeDataNew.obs.time_ms = storedRngMeasTime_ms[midIndex];
            }

            // limit the measured range to be no less than the on-ground range
            rangeDataNew.rng = MAX(storedRngMeas[midIndex], rngOnGnd);

            // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
            ekf_ring_buffer_push(&storedRange, RANGE_RING_BUFFER, &rangeDataNew);

            // indicate we have updated the measurement
            rngValidMeaTime_ms = imuSampleTime_ms;
        }
        else if (!takeOffDetected && ((imuSampleTime_ms - rngValidMeaTime_ms) > 200))
        {
            // before takeoff we assume on-ground range value if there is no data
            rangeDataNew.obs.time_ms = imuSampleTime_ms;
            rangeDataNew.rng = rngOnGnd;

            // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
            ekf_ring_buffer_push(&storedRange, RANGE_RING_BUFFER, &rangeDataNew);

            // indicate we have updated the measurement
            rngValidMeaTime_ms = imuSampleTime_ms;
        }
    }
}

// write the raw optical flow measurements this needs to be called externally.
void writeOptFlowMeas(const uint8_t rawFlowQuality, const fpVector2_t rawFlowRates, const fpVector2_t rawGyroRates, const fpVector3_t posOffset, float heightOverride)
{
    // The raw measurements need to be optical flow rates in radians/second averaged across the time since the last update
    // The PX4Flow sensor outputs flow rates with the following axis and sign conventions:
    // A positive X rate is produced by a positive sensor rotation about the X axis
    // A positive Y rate is produced by a positive sensor rotation about the Y axis
    // This filter uses a different definition of optical flow rates to the sensor with a positive optical flow rate produced by a
    // negative rotation about that axis. For example a positive rotation of the flight vehicle about its X (roll) axis would produce a negative X flow rate
    flowMeaTime_ms = imuSampleTime_ms;

    // calculate bias errors on flow sensor gyro rates, but protect against spikes in data
    // reset the accumulated body delta angle and time
    // don't do the calculation if not enough time lapsed for a reliable body rate measurement
    if (delTimeOF > 0.01f)
    {
        flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrainf((rawGyroRates.x - delAngBodyOF.x / delTimeOF), -0.1f, 0.1f);
        flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrainf((rawGyroRates.y - delAngBodyOF.y / delTimeOF), -0.1f, 0.1f);
        vectorZero(&delAngBodyOF);
        delTimeOF = 0.0f;
    }

    // by definition if this function is called, then flow measurements have been provided so we
    // need to run the optical flow takeoff detection
    detectOptFlowTakeoff();

    // calculate rotation matrices at mid sample time for flow observations
    quaternionToRotationMatrix(ekfStates.stateStruct.quat, &Tbn_flow);

    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && calc_length_pythagorean_2D(rawFlowRates.x, rawFlowRates.y) < 4.2f && calc_length_pythagorean_2D(rawGyroRates.x, rawGyroRates.y) < 4.2f)
    {
        // correct flow sensor body rates for bias and write
        ofDataNew.bodyRadXYZ.x = rawGyroRates.x - flowGyroBias.x;
        ofDataNew.bodyRadXYZ.y = rawGyroRates.y - flowGyroBias.y;

        // the sensor interface doesn't provide a z axis rate so use the rate from the nav sensor instead
        if (delTimeOF > 0.001f)
        {
            // first preference is to use the rate averaged over the same sampling period as the flow sensor
            ofDataNew.bodyRadXYZ.z = delAngBodyOF.z / delTimeOF;
        }
        else if (imuDataNew.delAngDT > 0.001f)
        {
            // second preference is to use most recent IMU data
            ofDataNew.bodyRadXYZ.z = imuDataNew.delAng.z / imuDataNew.delAngDT;
        }
        else
        {
            // third preference is use zero
            ofDataNew.bodyRadXYZ.z = 0.0f;
        }

        // write uncorrected flow rate measurements
        // note correction for different axis and sign conventions used by the px4flow sensor
        ofDataNew.flowRadXY.x = -rawFlowRates.x; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        ofDataNew.flowRadXY.y = -rawFlowRates.y; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)

        // write the flow sensor position in body frame
        ofDataNew.body_offset = posOffset;

        // write the flow sensor height override
        ofDataNew.heightOverride = heightOverride;

        // write flow rate measurements corrected for body rates
        ofDataNew.flowRadXYcomp.x = ofDataNew.flowRadXY.x + ofDataNew.bodyRadXYZ.x;
        ofDataNew.flowRadXYcomp.y = ofDataNew.flowRadXY.y + ofDataNew.bodyRadXYZ.y;

        // record time last observation was received so we can detect loss of data elsewhere
        flowValidMeaTime_ms = imuSampleTime_ms;

        // estimate sample time of the measurement
        ofDataNew.obs.time_ms = imuSampleTime_ms - ekfParam._flowDelay_ms - ekfInternalParam.flowTimeDeltaAvg_ms / 2;

        // Correct for the average intersampling delay due to the filter updaterate
        ofDataNew.obs.time_ms -= localFilterTimeStep_ms / 2;

        // Prevent time delay exceeding age of oldest IMU data in the buffer
        ofDataNew.obs.time_ms = MAX(ofDataNew.obs.time_ms, imuDataDelayed.time_ms);

        // Save data to buffer
        ekf_ring_buffer_push(&storedOF, OPTFLOW_RING_BUFFER, &ofDataNew);

        // Check for data at the fusion time horizon
        flowDataToFuse = ekf_ring_buffer_recall(&storedOF, OPTFLOW_RING_BUFFER, &ofDataDelayed, imuDataDelayed.time_ms);
    }
}

// check for new magnetometer data and update store measurements if available
void readMagData(void)
{
    if (!ekf_useCompass())
    {
        magSensorFailed = true;
        return;
    }

    // If we are a vehicle with a sideslip constraint to aid yaw estimation and we have timed out on our last avialable
    // magnetometer, then declare the magnetometers as failed for this flight
    if (magSensorFailed || (magTimeout && assume_zero_sideslip() && inFlight))
    {
        magSensorFailed = true;
        return;
    }

    if (ekfParam._ekfCompassLearn)
    {
        // while learning offsets keep all mag states reset
        InitialiseVariablesMag();
        wasLearningCompass_ms = imuSampleTime_ms;
    }
    else if (wasLearningCompass_ms != 0 && imuSampleTime_ms - wasLearningCompass_ms > 1000)
    {
        wasLearningCompass_ms = 0;
        // force a new yaw alignment 1s after learning completes. The delay is to ensure any buffered mag samples are discarded
        yawAlignComplete = false;
        InitialiseVariablesMag();
    }

    // do not accept new compass data faster than 14Hz (nominal rate is 10Hz) to prevent high processor loading
    // because magnetometer fusion is an expensive step and we could overflow the FIFO buffer
    if (ekf_useCompass() && compassLastUpdate() - lastMagUpdate_us > 70000)
    {
        // detect changes to magnetometer offset parameters and reset states
        fpVector3_t nowMagOffsets = {.v = {(compassConfig()->magZero.raw[X] / 1024 * compassConfig()->magGain[X]),
                                           (compassConfig()->magZero.raw[Y] / 1024 * compassConfig()->magGain[Y]),
                                           (compassConfig()->magZero.raw[Z] / 1024 * compassConfig()->magGain[Z])}};

        bool changeDetected = lastMagOffsetsValid &&
                              (nowMagOffsets.x != lastMagOffsets.x ||
                               nowMagOffsets.y != lastMagOffsets.y ||
                               nowMagOffsets.z != lastMagOffsets.z);

        if (changeDetected)
        {
            // zero the learned magnetometer bias states
            vectorZero(&ekfStates.stateStruct.body_magfield);
            // clear the measurement buffer
            ekf_ring_buffer_reset(&storedMag);
            // reset body mag variances on next CovariancePrediction. This copes with possible errors in the new offsets
            needMagBodyVarReset = true;
        }

        lastMagOffsets = nowMagOffsets;
        lastMagOffsetsValid = true;

        // store time of last measurement update
        lastMagUpdate_us = compassLastUpdate();

        // Magnetometer data at the current time horizon
        mag_elements_t magDataNew;

        // estimate of time magnetometer measurement was taken, allowing for delays
        magDataNew.obs.time_ms = imuSampleTime_ms - ekfInternalParam.magDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        magDataNew.obs.time_ms -= localFilterTimeStep_ms / 2;

        // read compass data and scale to improve numerical conditioning
        fpVector3_t magField = getUncorrectedMagField();
        magDataNew.mag.x = magField.x * 0.001f;
        magDataNew.mag.y = magField.y * 0.001f;
        magDataNew.mag.z = magField.z * 0.001f;

        // save magnetometer measurement to buffer to be fused later
        ekf_ring_buffer_push(&storedMag, MAG_RING_BUFFER, &magDataNew);

        // remember time we read compass, to detect compass sensor failure
        lastMagRead_ms = imuSampleTime_ms;
    }
}

// read the delta velocity and corresponding time interval from the IMU
void readDeltaVelocity(fpVector3_t *dVel, float *dVel_dt)
{
    const float acc_dt = (float)getTaskDeltaTime(TASK_PID) * 1.0e-6f;

    if (!accIntegratorInit)
    {
        accel_integrator.reset_interval_min = 1.0f / (float)ekfInternalParam.imuTimeHz;
        accel_integrator.reset_samples_min = 8;
        accIntegratorInit = true;
    }

    if (!integrator_reset_and_get_integral(&accel_integrator, dVel, dVel_dt))
    {
        fpVector3_t get_acc;
        accGetMeasuredAcceleration(&get_acc);
        // convert the accel in body frame in cm/s to m/s
        get_acc.x = get_acc.x * 0.01f;
        get_acc.y = get_acc.y * 0.01f;
        get_acc.z = get_acc.z * 0.01f;
        integrator_put(&accel_integrator, get_acc, acc_dt);
    }
}

// read the delta angle and corresponding time interval from the IMU
void readDeltaAngle(fpVector3_t *dAng, float *dAng_dt)
{
    const float gyro_dt = (float)getTaskDeltaTime(TASK_PID) * 1.0e-6f;

    if (!gyroIntegratorInit)
    {
        gyro_integrator.base.reset_interval_min = 1.0f / (float)ekfInternalParam.imuTimeHz;
        gyro_integrator.base.reset_samples_min = 8;
        gyroIntegratorInit = true;
    }

    if (!integrator_coning_reset_and_get_integral(&gyro_integrator, dAng, dAng_dt))
    {
        fpVector3_t get_gyro;
        gyroGetMeasuredRotationRate(&get_gyro);
        integrator_coning_put(&gyro_integrator, get_gyro, gyro_dt);
    }
}

/*
 *  Read IMU delta angle and delta velocity measurements and downsample to 100Hz
 *  for storage in the data buffers used by the EKF. If the IMU data arrives at
 *  lower rate than 100Hz, then no downsampling or upsampling will be performed.
 *  Downsampling is done using a method that does not introduce coning or sculling
 *  errors.
 */
void readIMUData(void)
{
    dtIMUavg = 1.0f / (float)ekfInternalParam.imuTimeHz;

    // Get delta velocity data from accelerometer
    readDeltaVelocity(&imuDataNew.delVel, &imuDataNew.delVelDT);

    // Get delta angle data from gyro
    readDeltaAngle(&imuDataNew.delAng, &imuDataNew.delAngDT);

    // Get current time stamp
    imuDataNew.time_ms = imuSampleTime_ms;

    // Accumulate the measurement time interval for the delta velocity and angle data
    imuDataDownSampledNew.delAngDT += imuDataNew.delAngDT;
    imuDataDownSampledNew.delVelDT += imuDataNew.delVelDT;

    // Rotate quaternon atitude from previous to new and normalise.
    // Accumulation using quaternions prevents introduction of coning errors due to downsampling
    quaternion_rotate(&imuQuatDownSampleNew, imuDataNew.delAng);
    quaternion_normalize(&imuQuatDownSampleNew);

    // Rotate the latest delta velocity into body frame at the start of accumulation
    fpMat3_t deltaRotMat;
    quaternionToRotationMatrix(imuQuatDownSampleNew, &deltaRotMat);

    // Apply the delta velocity to the delta velocity accumulator
    fpVector3_t mulResult = multiplyMatrixByVector(deltaRotMat, imuDataNew.delVel);
    imuDataDownSampledNew.delVel.x += mulResult.x;
    imuDataDownSampledNew.delVel.y += mulResult.y;
    imuDataDownSampledNew.delVel.z += mulResult.z;

    // Keep track of the number of IMU frames since the last state prediction
    framesSincePredict++;

    /*
     * If the target EKF time step has been accumulated, and the frontend has allowed start of a new predict cycle,
     * then store the accumulated IMU data to be used by the state prediction, ignoring the frontend permission if more
     * than twice the target time has lapsed. Adjust the target EKF step time threshold to allow for timing jitter in the
     * IMU data.
     */
    if ((dtIMUavg * (float)framesSincePredict >= (EKF_TARGET_DT - (dtIMUavg * 0.5f)) && startPredictEnabled) ||
        (dtIMUavg * (float)framesSincePredict >= 2.0f * EKF_TARGET_DT))
    {
        // convert the accumulated quaternion to an equivalent delta angle
        quaternionToAxisAngleV(imuQuatDownSampleNew, &imuDataDownSampledNew.delAng);

        // time stamp the data
        imuDataDownSampledNew.time_ms = imuSampleTime_ms;

        // write data to the FIFO IMU buffer
        ekf_imu_buffer_push_youngest_element(&storedIMU, &imuDataDownSampledNew);

        // calculate the achieved average time step rate for the EKF
        const float dtNow = constrainf(0.5f * (imuDataDownSampledNew.delAngDT + imuDataDownSampledNew.delVelDT), 0.0f, 10.0f * EKF_TARGET_DT);
        dtEkfAvg = 0.98f * dtEkfAvg + 0.02f * dtNow;

        // zero the accumulated IMU data and quaternion
        vectorZero(&imuDataDownSampledNew.delAng);
        vectorZero(&imuDataDownSampledNew.delVel);
        imuDataDownSampledNew.delAngDT = 0.0f;
        imuDataDownSampledNew.delVelDT = 0.0f;
        imuQuatDownSampleNew.q0 = 1.0f;
        imuQuatDownSampleNew.q3 = imuQuatDownSampleNew.q2 = imuQuatDownSampleNew.q1 = 0.0f;

        // reset the counter used to let the frontend know how many frames have elapsed since we started a new update cycle
        framesSincePredict = 0;

        // set the flag to let the filter know it has new IMU data and needs to run
        runUpdates = true;

        // extract the oldest available data from the FIFO buffer
        ekf_imu_buffer_get_oldest_element(&storedIMU, &imuDataDelayed);

        // protect against delta time going to zero
        // TODO - check if calculations can tolerate 0
        float minDT = 0.1f * dtEkfAvg;
        imuDataDelayed.delAngDT = MAX(imuDataDelayed.delAngDT, minDT);
        imuDataDelayed.delVelDT = MAX(imuDataDelayed.delVelDT, minDT);

        // correct the extracted IMU data for sensor errors
        delAngCorrected = imuDataDelayed.delAng;
        delVelCorrected = imuDataDelayed.delVel;
        correctDeltaAngle(&delAngCorrected, imuDataDelayed.delAngDT);
        correctDeltaVelocity(&delVelCorrected, imuDataDelayed.delVelDT);
    }
    else
    {
        // we don't have new IMU data in the buffer so don't run filter updates on this time step
        runUpdates = false;
    }
}

// 120ms for UBLOX7, UBLOXM8, UBLOXM9 and UBLOXM10
// 110ms for GPS MSP
timeMs_t ekf_getGPSDelay(void)
{
    return gpsConfig()->provider == GPS_MSP ? 110 : 120;
}

// check for new valid GPS data and update stored measurement if available
void readGpsData(void)
{
    if (ekfParam._fusionModeGPS == 3)
    {
        // don't read GPS data if GPS usage disabled
        return;
    }

    // check for new GPS data do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    if (sensors(SENSOR_GPS) && gpsState.lastMessageMs - lastTimeGpsReceived_ms > 70)
    {
        if (gpsSol.fixType >= GPS_FIX_3D)
        {
            // store fix time from previous read
            const uint32_t secondLastGpsTime_ms = lastTimeGpsReceived_ms;

            // get current fix time
            lastTimeGpsReceived_ms = gpsState.lastMessageMs;

            // estimate when the GPS fix was valid, allowing for GPS processing and other delays
            // ideally we should be using a timing signal from the GPS receiver to set this time
            gpsDataNew.obs.time_ms = lastTimeGpsReceived_ms - ekf_getGPSDelay();

            // Correct for the average intersampling delay due to the filter updaterate
            gpsDataNew.obs.time_ms -= localFilterTimeStep_ms / 2;

            // Prevent time delay exceeding age of oldest IMU data in the buffer
            gpsDataNew.obs.time_ms = MAX(gpsDataNew.obs.time_ms, imuDataDelayed.time_ms);

            // read the NED velocity from the GPS
            gpsDataNew.vel.x = CENTIMETERS_TO_METERS(gpsSol.velNED[X]);
            gpsDataNew.vel.y = CENTIMETERS_TO_METERS(gpsSol.velNED[Y]);
            gpsDataNew.vel.z = CENTIMETERS_TO_METERS(gpsSol.velNED[Z]);

            // Use the speed and position accuracy from the GPS if available, otherwise set it to zero.
            // Apply a decaying envelope filter with a 5 second time constant to the raw accuracy data
            float alpha = constrainf(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms), 0.0f, 1.0f);
            gpsSpdAccuracy *= (1.0f - alpha);
            float gpsSpdAccRaw = CENTIMETERS_TO_METERS(gpsSol.speed_accuracy);
            if (!gpsSol.speed_accuracy)
            {
                gpsSpdAccuracy = 0.0f;
            }
            else
            {
                gpsSpdAccuracy = MAX(gpsSpdAccuracy, gpsSpdAccRaw);
                gpsSpdAccuracy = MIN(gpsSpdAccuracy, 50.0f);
                gpsSpdAccuracy = MAX(gpsSpdAccuracy, ekfParam._gpsHorizVelNoise);
            }
            gpsPosAccuracy *= (1.0f - alpha);
            float gpsPosAccRaw = gpsSolDRV.eph / 10;
            if (!gpsSol.flags.validVelNE)
            {
                gpsPosAccuracy = 0.0f;
            }
            else
            {
                gpsPosAccuracy = MAX(gpsPosAccuracy, gpsPosAccRaw);
                gpsPosAccuracy = MIN(gpsPosAccuracy, 100.0f);
                gpsPosAccuracy = MAX(gpsPosAccuracy, ekfParam._gpsHorizPosNoise);
            }
            gpsHgtAccuracy *= (1.0f - alpha);
            float gpsHgtAccRaw = gpsSolDRV.epv / 10;
            if (!gpsSol.flags.validVelD)
            {
                gpsHgtAccuracy = 0.0f;
            }
            else
            {
                gpsHgtAccuracy = MAX(gpsHgtAccuracy, gpsHgtAccRaw);
                gpsHgtAccuracy = MIN(gpsHgtAccuracy, 100.0f);
                gpsHgtAccuracy = MAX(gpsHgtAccuracy, 1.5f * ekfParam._gpsHorizPosNoise);
            }

            // check if we have enough GPS satellites and increase the gps noise scaler if we don't
            if (gpsSol.numSat >= 6 && (PV_AidingMode == AID_ABSOLUTE))
            {
                gpsNoiseScaler = 1.0f;
            }
            else if (gpsSol.numSat == 5 && (PV_AidingMode == AID_ABSOLUTE))
            {
                gpsNoiseScaler = 1.4f;
            }
            else
            {
                // <= 4 satellites or in constant position mode
                gpsNoiseScaler = 2.0f;
            }

            // Check if GPS can output vertical velocity, if it is allowed to be used, and set GPS fusion mode accordingly
            if (gpsSol.flags.validVelD && ekfParam._fusionModeGPS == 0)
            {
                useGpsVertVel = true;
            }
            else
            {
                useGpsVertVel = false;
            }

            // Monitor quality of the GPS velocity data both before and after alignment. This updates
            // GpsGoodToAlign class variable
            calcGpsGoodToAlign();

            // Post-alignment checks
            calcGpsGoodForFlight();

            // see if we can get origin from frontend
            if (!validOrigin && ekf_common_origin_valid)
            {

                if (!setOrigin(&common_EKF_origin))
                {
                    return;
                }
            }

            // Read the GPS location in WGS-84 lat,long,height coordinates
            gpsLocation_t gpsloc = {.lat = gpsSol.llh.lat, .lon = gpsSol.llh.lon, .alt = gpsSol.llh.alt};

            // Set the EKF origin and magnetic field declination if not previously set  and GPS checks have passed
            if (gpsGoodToAlign && !validOrigin)
            {
                gpsLocation_t gpsloc_fieldelevation = gpsloc;
                // if flying, correct for height change from takeoff so that the origin is at field elevation
                if (inFlight)
                {
                    gpsloc_fieldelevation.alt += (int32_t)(100.0f * ekfStates.stateStruct.position.z);
                }

                if (!setOrigin(&gpsloc_fieldelevation))
                {
                    return;
                }

                // set the NE earth magnetic field states using the published declination
                // and set the corresponding variances and covariances
                alignMagStateDeclination();

                // Set the height of the NED origin
                ekfGpsRefHgt = 0.01f * gpsloc.alt + outputDataNew.position.z;

                // Set the uncertainty of the GPS origin height
                ekfOriginHgtVar = sq(gpsHgtAccuracy);
            }

            if (gpsGoodToAlign && !have_table_earth_field)
            {
                if (compassIsCalibrationComplete() && positionEstimationConfig()->automatic_mag_declination)
                {
#ifndef SOLVE_AFTER
                    table_earth_field_ga = get_earth_field_ga(gpsloc);
#endif
                    table_declination = geoCalculateMagDeclination(&gpsloc);
                    have_table_earth_field = true;
                    if (ekfParam._mag_ef_limit > 0)
                    {
                        // initialise earth field from tables
                        ekfStates.stateStruct.earth_magfield = table_earth_field_ga;
                    }
                }
            }

            // convert GPS measurements to local NED and save to buffer to be fused later if we have a valid origin
            if (validOrigin)
            {
                gpsDataNew.pos = get_distance_NE(EKF_origin, gpsloc);
                if ((ekfParam._originHgtMode & (1 << HGT_SOURCE_GPS)) == 0)
                {
                    gpsDataNew.hgt = (float)(0.01f * gpsloc.alt - ekfGpsRefHgt);
                }
                else
                {
                    gpsDataNew.hgt = 0.01 * (gpsloc.alt - EKF_origin.alt);
                }
                ekf_ring_buffer_push(&storedGPS, GPS_RING_BUFFER, &gpsDataNew);
                // declare GPS available for use
                gpsNotAvailable = false;
            }
        }
        else
        {
            sendEKFLogMessage("EKF waiting for 3D fix");
        }
    }
}

// check for new pressure altitude measurement data and update stored measurement if available
void readBaroData(void)
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    if (sensors(SENSOR_BARO) && baro.calibrationFinished && US2MS(posEstimator.baro.lastUpdateTime) - lastBaroReceived_ms > 70)
    {
        baroDataNew.hgt = CENTIMETERS_TO_METERS(posEstimator.baro.rawAlt - posEstimator.baro.altOffSet);

        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
        if (get_takeoff_expected() && !assume_zero_sideslip())
        {
            baroDataNew.hgt = MAX(baroDataNew.hgt, meaHgtAtTakeOff);
        }

        // time stamp used to check for new measurement
        lastBaroReceived_ms = US2MS(posEstimator.baro.lastUpdateTime);

        // estimate of time height measurement was taken, allowing for delays
        baroDataNew.obs.time_ms = lastBaroReceived_ms - ekfParam._hgtDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        baroDataNew.obs.time_ms -= localFilterTimeStep_ms / 2;

        // Prevent time delay exceeding age of oldest IMU data in the buffer
        baroDataNew.obs.time_ms = MAX(baroDataNew.obs.time_ms, imuDataDelayed.time_ms);

        // save baro measurement to buffer to be fused later
        ekf_ring_buffer_push(&storedBaro, BARO_RING_BUFFER, &baroDataNew);
    }
}

// calculate filtered offset between baro height measurement and EKF height estimate
// offset should be subtracted from baro measurement to match filter estimate
// offset is used to enable reversion to baro from alternate height data source
void calcFiltBaroOffset(void)
{
    // Apply a first order LPF with spike protection
    baroHgtOffset += 0.1f * constrainf(baroDataDelayed.hgt + ekfStates.stateStruct.position.z - baroHgtOffset, -5.0f, 5.0f);
}

// correct the height of the EKF origin to be consistent with GPS Data using a Bayes filter.
void correctEkfOriginHeight(void)
{
    // Estimate the WGS-84 height of the EKF's origin using a Bayes filter

    // calculate the variance of our a-priori estimate of the ekf origin height
    float deltaTime = constrainf(0.001f * (imuDataDelayed.time_ms - lastOriginHgtTime_ms), 0.0f, 1.0f);
    if (activeHgtSource == HGT_SOURCE_BARO)
    {
        // Use the baro drift rate
        const float baroDriftRate = 0.05f;
        ekfOriginHgtVar += sq(baroDriftRate * deltaTime);
    }
    else if (activeHgtSource == HGT_SOURCE_RNG)
    {
        // use the worse case expected terrain gradient and vehicle horizontal speed
        const float maxTerrGrad = 0.25f;
        ekfOriginHgtVar += sq(maxTerrGrad * calc_length_pythagorean_2D(ekfStates.stateStruct.velocity.x, ekfStates.stateStruct.velocity.y) * deltaTime);
    }
    else
    {
        // by definition our height source is absolute so cannot run this filter
        return;
    }

    lastOriginHgtTime_ms = imuDataDelayed.time_ms;

    // calculate the observation variance assuming EKF error relative to datum is independent of GPS observation error
    // when not using GPS as height source
    float originHgtObsVar = sq(gpsHgtAccuracy) + P[8][8];

    // calculate the correction gain
    float gain = ekfOriginHgtVar / (ekfOriginHgtVar + originHgtObsVar);

    // calculate the innovation
    float innovation = -ekfStates.stateStruct.position.z - gpsDataDelayed.hgt;

    // check the innovation variance ratio
    float ratio = sq(innovation) / (ekfOriginHgtVar + originHgtObsVar);

    // correct the EKF origin and variance estimate if the innovation is less than 5-sigma
    if (ratio < 25.0f && gpsAccuracyGood)
    {
        ekfGpsRefHgt -= (gain * innovation);
        ekfOriginHgtVar -= MAX(gain * ekfOriginHgtVar, 0.0f);
    }
}

// check for new airspeed data and update stored measurements if available
void readAirSpdData(void)
{
    // if airspeed reading is valid and is set by the user to be used and has been updated then
    // we take a new reading, convert from EAS to TAS and set the flag letting other functions
    // know a new measurement is available
    if (useAirspeed() && pitot.lastSeenHealthyMs != timeTasReceived_ms)
    {
        tasDataNew.tas = CENTIMETERS_TO_METERS(getAirspeedEstimate());
        timeTasReceived_ms = pitot.lastSeenHealthyMs;
        tasDataNew.obs.time_ms = timeTasReceived_ms - ekfInternalParam.tasDelay_ms;

        // Correct for the average intersampling delay due to the filter update rate
        tasDataNew.obs.time_ms -= localFilterTimeStep_ms / 2;

        // Save data into the buffer to be fused when the fusion time horizon catches up with it
        ekf_ring_buffer_push(&storedTAS, TAS_RING_BUFFER, &tasDataNew);
    }
    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    tasDataToFuse = ekf_ring_buffer_recall(&storedTAS, TAS_RING_BUFFER, &tasDataDelayed, imuDataDelayed.time_ms);
}

// return declination in radians
float MagDeclination(void)
{
    // if we are using the WMM tables then use the table declination
    // to ensure consistency with the table mag field. Otherwise use
    // the declination from the compass library
    if (have_table_earth_field && ekfParam._mag_ef_limit > 0)
    {
        return table_declination;
    }

    if (!ekf_useCompass())
    {
        return 0.0f;
    }

    return compassConfig()->mag_declination;
}