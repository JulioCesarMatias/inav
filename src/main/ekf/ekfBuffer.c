#include "ekf/ekfBuffer.h"

static bool init_buffer_parent(ekf_ring_buffer *buffer, uint8_t sensor_buffer_num)
{
    bool ret;

    switch (sensor_buffer_num)
    {
    case GPS_RING_BUFFER:
        if (buffer->buffer.gps_buffer != NULL)
        {
            free(buffer->buffer.gps_buffer);
        }
        buffer->buffer.gps_buffer = calloc(buffer->size, buffer->elsize);
        ret = buffer->buffer.gps_buffer == NULL ? false : true;
        break;

    case MAG_RING_BUFFER:
        if (buffer->buffer.mag_buffer != NULL)
        {
            free(buffer->buffer.mag_buffer);
        }
        buffer->buffer.mag_buffer = calloc(buffer->size, buffer->elsize);
        ret = buffer->buffer.mag_buffer == NULL ? false : true;
        break;

    case BARO_RING_BUFFER:
        if (buffer->buffer.baro_buffer != NULL)
        {
            free(buffer->buffer.baro_buffer);
        }
        buffer->buffer.baro_buffer = calloc(buffer->size, buffer->elsize);
        ret = buffer->buffer.baro_buffer == NULL ? false : true;
        break;

    case RANGE_RING_BUFFER:
        if (buffer->buffer.range_buffer != NULL)
        {
            free(buffer->buffer.range_buffer);
        }
        buffer->buffer.range_buffer = calloc(buffer->size, buffer->elsize);
        ret = buffer->buffer.range_buffer == NULL ? false : true;
        break;

    case TAS_RING_BUFFER:
        if (buffer->buffer.tas_buffer != NULL)
        {
            free(buffer->buffer.tas_buffer);
        }
        buffer->buffer.tas_buffer = calloc(buffer->size, buffer->elsize);
        ret = buffer->buffer.tas_buffer == NULL ? false : true;
        break;

    case OPTFLOW_RING_BUFFER:
        if (buffer->buffer.of_buffer != NULL)
        {
            free(buffer->buffer.of_buffer);
        }
        buffer->buffer.of_buffer = calloc(buffer->size, buffer->elsize);
        ret = buffer->buffer.of_buffer == NULL ? false : true;
        break;
    }

    return ret;
}

bool ekf_ring_buffer_init_size(ekf_ring_buffer *buffer, uint8_t sensor_buffer_num, uint8_t _size, uint8_t _elsize)
{
    buffer->size = _size;
    buffer->elsize = _elsize;

    if (!init_buffer_parent(buffer, sensor_buffer_num))
    {
        return false;
    }

    ekf_ring_buffer_reset(buffer);

    return true;
}

static uint32_t ekf_ring_buffer_time_ms(ekf_ring_buffer *buffer, uint8_t sensor_buffer_num)
{
    EKF_obs_element_t *el;

    switch (sensor_buffer_num)
    {
    case GPS_RING_BUFFER:
        el = (EKF_obs_element_t *)(((uint8_t *)buffer->buffer.gps_buffer) + buffer->oldest * buffer->elsize);
        break;

    case MAG_RING_BUFFER:
        el = (EKF_obs_element_t *)(((uint8_t *)buffer->buffer.mag_buffer) + buffer->oldest * buffer->elsize);
        break;

    case BARO_RING_BUFFER:
        el = (EKF_obs_element_t *)(((uint8_t *)buffer->buffer.baro_buffer) + buffer->oldest * buffer->elsize);
        break;

    case RANGE_RING_BUFFER:
        el = (EKF_obs_element_t *)(((uint8_t *)buffer->buffer.range_buffer) + buffer->oldest * buffer->elsize);
        break;

    case TAS_RING_BUFFER:
        el = (EKF_obs_element_t *)(((uint8_t *)buffer->buffer.tas_buffer) + buffer->oldest * buffer->elsize);
        break;

    case OPTFLOW_RING_BUFFER:
        el = (EKF_obs_element_t *)(((uint8_t *)buffer->buffer.of_buffer) + buffer->oldest * buffer->elsize);
        break;
    }

    return el->time_ms;
}

static void select_buffer_recall(ekf_ring_buffer *buffer, void *element, uint8_t sensor_buffer_num)
{
    switch (sensor_buffer_num)
    {
    case GPS_RING_BUFFER:
        memcpy(element, (((uint8_t *)buffer->buffer.gps_buffer) + buffer->oldest * buffer->elsize), buffer->elsize);
        break;

    case MAG_RING_BUFFER:
        memcpy(element, (((uint8_t *)buffer->buffer.mag_buffer) + buffer->oldest * buffer->elsize), buffer->elsize);
        break;

    case BARO_RING_BUFFER:
        memcpy(element, (((uint8_t *)buffer->buffer.baro_buffer) + buffer->oldest * buffer->elsize), buffer->elsize);
        break;

    case RANGE_RING_BUFFER:
        memcpy(element, (((uint8_t *)buffer->buffer.range_buffer) + buffer->oldest * buffer->elsize), buffer->elsize);
        break;

    case TAS_RING_BUFFER:
        memcpy(element, (((uint8_t *)buffer->buffer.tas_buffer) + buffer->oldest * buffer->elsize), buffer->elsize);
        break;

    case OPTFLOW_RING_BUFFER:
        memcpy(element, (((uint8_t *)buffer->buffer.of_buffer) + buffer->oldest * buffer->elsize), buffer->elsize);
        break;
    }
}

bool ekf_ring_buffer_recall(ekf_ring_buffer *buffer, uint8_t sensor_buffer_num, void *element, const uint32_t sample_time_ms)
{
    bool ret = false;

    while (buffer->count > 0)
    {
        const uint32_t toldest = ekf_ring_buffer_time_ms(buffer, sensor_buffer_num);
        const int32_t dt = sample_time_ms - toldest;
        const bool matches = dt >= 0 && dt < 100;
        if (matches)
        {
            select_buffer_recall(buffer, element, sensor_buffer_num);
            ret = true;
        }
        if (dt < 0)
        {
            break;
        }
        buffer->count--;
        buffer->oldest = (buffer->oldest + 1) % buffer->size;
    }

    return ret;
}

static bool select_buffer_push(ekf_ring_buffer *buffer, const void *element, uint8_t sensor_buffer_num)
{
    const uint8_t head = (buffer->oldest + buffer->count) % buffer->size;

    switch (sensor_buffer_num)
    {
    case GPS_RING_BUFFER:
        if (buffer->buffer.gps_buffer == NULL)
        {
            return false;
        }
        memcpy((((uint8_t *)buffer->buffer.gps_buffer) + head * buffer->elsize), element, buffer->elsize);
        break;

    case MAG_RING_BUFFER:
        if (buffer->buffer.mag_buffer == NULL)
        {
            return false;
        }
        memcpy((((uint8_t *)buffer->buffer.mag_buffer) + head * buffer->elsize), element, buffer->elsize);
        break;

    case BARO_RING_BUFFER:
        if (buffer->buffer.baro_buffer == NULL)
        {
            return false;
        }
        memcpy((((uint8_t *)buffer->buffer.baro_buffer) + head * buffer->elsize), element, buffer->elsize);
        break;

    case RANGE_RING_BUFFER:
        if (buffer->buffer.range_buffer == NULL)
        {
            return false;
        }
        memcpy((((uint8_t *)buffer->buffer.range_buffer) + head * buffer->elsize), element, buffer->elsize);
        break;

    case TAS_RING_BUFFER:
        if (buffer->buffer.tas_buffer == NULL)
        {
            return false;
        }
        memcpy((((uint8_t *)buffer->buffer.tas_buffer) + head * buffer->elsize), element, buffer->elsize);
        break;

    case OPTFLOW_RING_BUFFER:
        if (buffer->buffer.of_buffer == NULL)
        {
            return false;
        }
        memcpy((((uint8_t *)buffer->buffer.of_buffer) + head * buffer->elsize), element, buffer->elsize);
        break;
    }

    return true;
}

void ekf_ring_buffer_push(ekf_ring_buffer *buffer, uint8_t sensor_buffer_num, const void *element)
{
    if (!select_buffer_push(buffer, element, sensor_buffer_num))
    {
        return;
    }

    if (buffer->count < buffer->size)
    {
        buffer->count++;
    }
    else
    {
        buffer->oldest = (buffer->oldest + 1) % buffer->size;
    }
}

void ekf_ring_buffer_reset(ekf_ring_buffer *buffer)
{
    buffer->count = 0;
    buffer->oldest = 0;
}

static bool init_imu_buffer_parent(ekf_imu_buffer *buffer, uint8_t sensor_buffer_num)
{
    bool ret;

    switch (sensor_buffer_num)
    {
    case IMU_RING_BUFFER:
        if (buffer->buffer.imu_buffer != NULL)
        {
            free(buffer->buffer.imu_buffer);
        }
        buffer->buffer.imu_buffer = calloc(buffer->size, buffer->elsize);
        ret = buffer->buffer.imu_buffer == NULL ? false : true;
        break;

    case OUTPUT_RING_BUFFER:
        if (buffer->buffer.output_buffer != NULL)
        {
            free(buffer->buffer.output_buffer);
        }
        buffer->buffer.output_buffer = calloc(buffer->size, buffer->elsize);
        ret = buffer->buffer.output_buffer == NULL ? false : true;
        break;
    }

    return ret;
}

bool ekf_imu_buffer_init_size(ekf_imu_buffer *buffer, uint8_t sensor_buffer_num, uint8_t size, uint8_t _elsize)
{

    buffer->elsize = _elsize;
    buffer->size = size;

    if (!init_imu_buffer_parent(buffer, sensor_buffer_num))
    {
        return false;
    }

    buffer->youngest = 0;
    buffer->oldest = 0;

    return true;
}

void ekf_imu_buffer_push_youngest_element(ekf_imu_buffer *buffer, imu_elements_t *element)
{
    if (buffer->buffer.imu_buffer == NULL)
    {
        return;
    }

    buffer->youngest = (buffer->youngest + 1) % buffer->size;
    memcpy((((uint8_t *)buffer->buffer.imu_buffer) + buffer->youngest * buffer->elsize), element, buffer->elsize);
    buffer->oldest = (buffer->youngest + 1) % buffer->size;

    if (buffer->oldest == 0)
    {
        buffer->filled = true;
    }
}

bool ekf_imu_buffer_is_filled(const ekf_imu_buffer *buffer)
{
    return buffer->filled;
}

void ekf_imu_buffer_get_oldest_element(const ekf_imu_buffer *buffer, imu_elements_t *element)
{
    if (buffer->buffer.imu_buffer == NULL)
    {
        memset(element, 0, buffer->elsize);
    }
    else
    {
        memcpy(element, (((uint8_t *)buffer->buffer.imu_buffer) + buffer->oldest * buffer->elsize), buffer->elsize);
    }
}

void ekf_imu_buffer_reset_history(ekf_imu_buffer *buffer, imu_elements_t *element)
{
    for (uint8_t index = 0; index < buffer->size; index++)
    {
        memcpy((((uint8_t *)buffer->buffer.imu_buffer) + index * buffer->elsize), element, buffer->elsize);
    }
}

void ekf_imu_buffer_reset(ekf_imu_buffer *buffer, uint8_t sensor_buffer_num)
{
    buffer->youngest = 0;
    buffer->oldest = 0;

    switch (sensor_buffer_num)
    {
    case IMU_RING_BUFFER:
        memset(buffer->buffer.imu_buffer, 0, buffer->size * buffer->elsize);
        break;

    case OUTPUT_RING_BUFFER:
        memset(buffer->buffer.output_buffer, 0, buffer->size * buffer->elsize);
        break;
    }
}

output_elements_t *ekf_output_buffer_get(const ekf_imu_buffer *buffer, uint8_t index)
{
    return (output_elements_t *)(((uint8_t *)buffer->buffer.output_buffer) + index * buffer->elsize);
}