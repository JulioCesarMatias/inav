/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "ekf/ekfCore.h"

bool ekf_ring_buffer_init_size(ekf_ring_buffer *buffer, uint8_t sensor_buffer_num, uint8_t _size, uint8_t _elsize);
bool ekf_ring_buffer_recall(ekf_ring_buffer *buffer, uint8_t sensor_buffer_num, void *element, const uint32_t sample_time_ms);
void ekf_ring_buffer_push(ekf_ring_buffer *buffer, uint8_t sensor_buffer_num, const void *element);
void ekf_ring_buffer_reset(ekf_ring_buffer *buffer);

bool ekf_imu_buffer_init_size(ekf_imu_buffer *buffer, uint8_t sensor_buffer_num, uint8_t size, uint8_t _elsize);
void ekf_imu_buffer_push_youngest_element(ekf_imu_buffer *buffer, imu_elements_t *element);
bool ekf_imu_buffer_is_filled(const ekf_imu_buffer *buffer);
void ekf_imu_buffer_get_oldest_element(const ekf_imu_buffer *buffer, imu_elements_t *element);
void ekf_imu_buffer_reset_history(ekf_imu_buffer *buffer, imu_elements_t *element);
void ekf_imu_buffer_reset(ekf_imu_buffer *buffer, uint8_t sensor_buffer_num);
output_elements_t *ekf_output_buffer_get(const ekf_imu_buffer *buffer, uint8_t index);
