#include "ekf/ekfBuffer.h"

bool ekf_ring_buffer_init_size(ekf_ring_buffer *buffer, uint8_t _size, uint8_t _elsize)
{
    if (buffer->buffer != NULL)
    {
        free(buffer->buffer);
    }

    buffer->elsize = _elsize;
    buffer->buffer = calloc(_size, buffer->elsize);

    if (buffer->buffer == NULL)
    {
        return false;
    }

    buffer->size = _size;
    ekf_ring_buffer_reset(buffer);

    return true;
}

bool ekf_ring_buffer_recall(ekf_ring_buffer *buffer, void *element, const uint32_t sample_time_ms)
{
    bool ret = false;

    while (buffer->count > 0)
    {
        const uint32_t toldest = *((uint32_t *)((uint8_t *)buffer->buffer + buffer->oldest * buffer->elsize));
        const int32_t dt = sample_time_ms - toldest;
        const bool matches = dt >= 0 && dt < 100;
        if (matches)
        {
            memcpy(element, (uint8_t *)buffer->buffer + buffer->oldest * buffer->elsize, buffer->elsize);
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

void ekf_ring_buffer_push(ekf_ring_buffer *buffer, const void *element)
{
    if (buffer->buffer == NULL)
    {
        return;
    }

    const uint8_t head = (buffer->oldest + buffer->count) % buffer->size;
    memcpy((uint8_t *)buffer->buffer + head * buffer->elsize, element, buffer->elsize);

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

bool ekf_imu_buffer_init_size(ekf_imu_buffer *buffer, uint32_t size, uint8_t _elsize)
{
    if (buffer->buffer != NULL)
    {
        free(buffer->buffer);
    }

    buffer->elsize = _elsize;
    buffer->buffer = calloc(size, buffer->elsize);

    if (buffer->buffer == NULL)
    {
        return false;
    }

    buffer->size = size;
    buffer->youngest = 0;
    buffer->oldest = 0;

    return true;
}

void ekf_imu_buffer_push_youngest_element(ekf_imu_buffer *buffer, imu_elements_t *element)
{
    if (buffer->buffer == NULL)
    {
        return;
    }

    buffer->youngest = (buffer->youngest + 1) % buffer->size;

    memcpy((uint8_t *)buffer->buffer + buffer->youngest * buffer->elsize, element, buffer->elsize);

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
    if (buffer->buffer == NULL)
    {
        memset(element, 0, buffer->elsize);
    }
    else
    {
        memcpy(element, (uint8_t *)buffer->buffer + buffer->oldest * buffer->elsize, buffer->elsize);
    }
}

void ekf_imu_buffer_reset_history(ekf_imu_buffer *buffer, imu_elements_t *element)
{
    for (uint8_t index = 0; index < buffer->size; index++)
    {
        memcpy((uint8_t *)buffer->buffer + index * buffer->elsize, element, buffer->elsize);
    }
}

void ekf_imu_buffer_reset(ekf_imu_buffer *buffer)
{
    buffer->youngest = 0;
    buffer->oldest = 0;
    memset(buffer->buffer, 0, buffer->size * buffer->elsize);
}

bool ekf_output_buffer_init_size(ekf_output_buffer *buffer, uint32_t size, uint8_t _elsize)
{
    if (buffer->buffer != NULL)
    {
        free(buffer->buffer);
    }

    buffer->elsize = _elsize;
    buffer->buffer = calloc(size, buffer->elsize);

    if (buffer->buffer == NULL)
    {
        return false;
    }

    buffer->size = size;
    buffer->youngest = 0;
    buffer->oldest = 0;

    return true;
}

void ekf_output_buffer_reset(ekf_output_buffer *buffer)
{
    buffer->youngest = 0;
    buffer->oldest = 0;
    memset(buffer->buffer, 0, buffer->size * buffer->elsize);
}

void *ekf_output_buffer_get(const ekf_output_buffer *buffer, uint8_t index)
{
    return (uint8_t *)buffer->buffer + index * buffer->elsize;
}