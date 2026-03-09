#pragma once

// Standard sensor definitions: SENSOR(name, hex_id, conversion_to_SI)
// Conversion is always multiplication: SI_value = raw * conversion

// clang-format off
#define STANDARD_SENSORS                                                                                            \
    /* Accelerometer (µg -> m/s²) */                                                                                \
    SENSOR(accX, 0x00, 1e-6 * G_TO_MS2)                                                                             \
    SENSOR(accY, 0x01, 1e-6 * G_TO_MS2)                                                                             \
    SENSOR(accZ, 0x02, 1e-6 * G_TO_MS2)                                                                             \
                                                                                                                    \
    /* Gyroscope (µdeg/s -> rad/s) */                                                                               \
    SENSOR(gyroX, 0x03, 1e-6 * DEG_TO_RAD)                                                                          \
    SENSOR(gyroY, 0x04, 1e-6 * DEG_TO_RAD)                                                                          \
    SENSOR(gyroZ, 0x05, 1e-6 * DEG_TO_RAD)                                                                          \
                                                                                                                    \
    /* Inclinometer (µg -> m/s²) */                                                                                 \
    SENSOR(inclX, 0x06, 1e-6 * G_TO_MS2)                                                                            \
    SENSOR(inclY, 0x07, 1e-6 * G_TO_MS2)                                                                            \
    SENSOR(inclZ, 0x08, 1e-6 * G_TO_MS2)                                                                            \
                                                                                                                    \
    /* Magnetometer (mGauss -> Tesla; 1 mGauss = 1e-7 T) */                                                         \
    SENSOR(magX, 0x0A, 1e-7)                                                                                        \
    SENSOR(magY, 0x0B, 1e-7)                                                                                        \
    SENSOR(magZ, 0x0C, 1e-7)                                                                                        \
                                                                                                                    \
    /* Barometer (mhPa -> Pa; 1 mhPa = 0.1 Pa) */                                                                   \
    SENSOR(barometer, 0x0D, 0.1)                                                                                    \
                                                                                                                    \
    /* Odometer (mm/s -> m/s) */                                                                                    \
    SENSOR(odometer_speed, 0x0E, 1e-3)                                                                              \
                                                                                                                    \
    /* GNSS Fixed relative position */                                                                              \
    SENSOR(gnss_fixed_relpos_itow   , 0x23, 1e-3)    /* ms -> s */                                                  \
    SENSOR(gnss_fixed_relpos_north  , 0x24, 1e-3)   /* mm -> m */                                                   \
    SENSOR(gnss_fixed_relpos_east   , 0x25, 1e-3)    /* mm -> m */                                                  \
    SENSOR(gnss_fixed_relpos_down   , 0x26, 1e-3)    /* mm -> m */                                                  \
                                                                                                                    \
    /* GNSS Moving relative position */                                                                             \
    SENSOR(gnss_moving_relpos_itow  , 0x27, 1e-3)   /* ms -> s */                                                   \
    SENSOR(gnss_moving_relpos_north , 0x28, 1e-3)  /* mm -> m */                                                    \
    SENSOR(gnss_moving_relpos_east  , 0x29, 1e-3)   /* mm -> m */                                                   \
    SENSOR(gnss_moving_relpos_down  , 0x2A, 1e-3)   /* mm -> m */                                                   \
                                                                                                                    \
    /* Horizontal position (1e-7 deg -> deg) */                                                                     \
    SENSOR(horizontal_position_latitude , 0x31, 1e-7)                                                               \
    SENSOR(horizontal_position_longitude, 0x32, 1e-7)                                                               \
                                                                                                                    \
    /* Horizontal velocity (mm/s -> m/s) */                                                                         \
    SENSOR(horizontal_velocity_north, 0x33, 1e-3)                                                                   \
    SENSOR(horizontal_velocity_east , 0x34, 1e-3)                                                                   \
                                                                                                                    \
    /* Vertical position/velocity (mm -> m, mm/s -> m/s) */                                                         \
    SENSOR(vertical_position        , 0x35, 1e-3)                                                                   \
    SENSOR(vertical_velocity_down   , 0x36, 1e-3)                                                                   \
                                                                                                                    \
    /* Euler angles (µdeg -> rad) */                                                                                \
    SENSOR(roll     , 0x37, 1e-6 * DEG_TO_RAD)                                                                      \
    SENSOR(pitch    , 0x38, 1e-6 * DEG_TO_RAD)                                                                      \
    SENSOR(heading  , 0x39, 1e-6 * DEG_TO_RAD)                                                                      \
                                                                                                                    \
    /* Corrected accelerometer (µg -> m/s²) */                                                                      \
    SENSOR(corrected_acc_x, 0x3A, 1e-6 * G_TO_MS2)                                                                  \
    SENSOR(corrected_acc_y, 0x3B, 1e-6 * G_TO_MS2)                                                                  \
    SENSOR(corrected_acc_z, 0x3C, 1e-6 * G_TO_MS2)                                                                  \
                                                                                                                    \
    /* Corrected gyroscope (µdeg/s -> rad/s) */                                                                     \
    SENSOR(corrected_gyro_x, 0x3D, 1e-6 * DEG_TO_RAD)                                                               \
    SENSOR(corrected_gyro_y, 0x3E, 1e-6 * DEG_TO_RAD)                                                               \
    SENSOR(corrected_gyro_z, 0x3F, 1e-6 * DEG_TO_RAD)                                                               \
                                                                                                                    \
    /* System time in milliseconds (raw) */                                                                         \
    SENSOR(system_time_ms, 0x42, 1.0)                                                                               \
                                                                                                                    \
    /* GNSS iTow (ms -> s) */                                                                                       \
    SENSOR(gnss1_itow, 0x43, 1e-3)                                                                                  \
    SENSOR(gnss2_itow, 0x44, 1e-3)                                                                                  \
                                                                                                                    \
    /* Sync (counter passthrough, µs -> s) */                                                                       \
    SENSOR(sync_in_count, 0x48, 1.0)                                                                                \
    SENSOR(sync_in_time , 0x49, 1e-6)                                                                               \
                                                                                                                    \
    /* UTC sub-second (µs -> s) */                                                                                  \
    SENSOR(utc_sub_second, 0x4A, 1e-6)                                                                              \
                                                                                                                    \
    /* Attitude quaternion (1e-6 scaled -> unitless) */                                                             \
    SENSOR(q_w, 0x4C, 1e-6)                                                                                         \
    SENSOR(q_x, 0x4D, 1e-6)                                                                                         \
    SENSOR(q_y, 0x4E, 1e-6)                                                                                         \
    SENSOR(q_z, 0x4F, 1e-6)                                                                                         \
                                                                                                                    \
    /* Rotation matrix (1e-6 scaled -> unitless) */                                                                 \
    SENSOR(rotation_matrix_11, 0x50, 1e-6)                                                                          \
    SENSOR(rotation_matrix_12, 0x51, 1e-6)                                                                          \
    SENSOR(rotation_matrix_13, 0x52, 1e-6)                                                                          \
    SENSOR(rotation_matrix_21, 0x53, 1e-6)                                                                          \
    SENSOR(rotation_matrix_22, 0x54, 1e-6)                                                                          \
    SENSOR(rotation_matrix_23, 0x55, 1e-6)                                                                          \
    SENSOR(rotation_matrix_31, 0x56, 1e-6)                                                                          \
    SENSOR(rotation_matrix_32, 0x57, 1e-6)                                                                          \
    SENSOR(rotation_matrix_33, 0x58, 1e-6)                                                                          \
                                                                                                                    \
    /* ECEF position (cm -> m) */                                                                                   \
    SENSOR(ecef_pos_x, 0x59, 1e-2)                                                                                  \
    SENSOR(ecef_pos_y, 0x5A, 1e-2)                                                                                  \
    SENSOR(ecef_pos_z, 0x5B, 1e-2)                                                                                  \
                                                                                                                    \
    /* System time in microseconds (raw) */                                                                         \
    SENSOR(system_time_us, 0x60, 1.0)                                                                               \
                                                                                                                    \
    /* Quality metrics: position (mm -> m) */                                                                       \
    SENSOR(quality_horizontal_pos_lat, 0x61, 1e-3)                                                                  \
    SENSOR(quality_horizontal_pos_lon, 0x62, 1e-3)                                                                  \
                                                                                                                    \
    /* Quality metrics: speed (mm/s -> m/s) */                                                                      \
    SENSOR(quality_horizontal_speed_north   , 0x63, 1e-3)                                                           \
    SENSOR(quality_horizontal_speed_east    , 0x64, 1e-3)                                                           \
                                                                                                                    \
    /* Quality metrics: vertical (mm -> m, mm/s -> m/s) */                                                          \
    SENSOR(quality_vertical_position, 0x65, 1e-3)                                                                   \
    SENSOR(quality_vertical_speed   , 0x66, 1e-3)                                                                   \
                                                                                                                    \
    /* Quality metrics: angles (µdeg -> rad) */                                                                     \
    SENSOR(quality_roll     , 0x67, 1e-6 * DEG_TO_RAD)                                                              \
    SENSOR(quality_pitch    , 0x68, 1e-6 * DEG_TO_RAD)                                                              \
    SENSOR(quality_heading  , 0x69, 1e-6 * DEG_TO_RAD)
// clang-format on
