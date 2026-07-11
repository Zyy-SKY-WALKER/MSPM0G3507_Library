/**
 * @file    drive_geometry.h
 * @brief   Shared differential-drive geometry and calibration constants.
 */

#ifndef DRIVE_GEOMETRY_H
#define DRIVE_GEOMETRY_H

#define DRIVE_PI                           (3.14159265F)
#define DRIVE_ENCODER_COUNTS_PER_REV       (10250U)

#define DRIVE_LEFT_WHEEL_DIAMETER_MM       (48.0F)
#define DRIVE_RIGHT_WHEEL_DIAMETER_MM      (48.0F)
#define DRIVE_TRACK_WIDTH_MM               (130.0F)

#define DRIVE_LEFT_MM_PER_COUNT            \
    ((DRIVE_PI * DRIVE_LEFT_WHEEL_DIAMETER_MM) \
        / (float)DRIVE_ENCODER_COUNTS_PER_REV)
#define DRIVE_RIGHT_MM_PER_COUNT           \
    ((DRIVE_PI * DRIVE_RIGHT_WHEEL_DIAMETER_MM) \
        / (float)DRIVE_ENCODER_COUNTS_PER_REV)

#define DRIVE_ODOMETRY_ENCODER_WEIGHT      (0.98F)
#define DRIVE_ODOMETRY_IMU_WEIGHT          \
    (1.0F - DRIVE_ODOMETRY_ENCODER_WEIGHT)

#endif
