#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "hal.h"

////////////////////////////////
//          SETTINGS          //
////////////////////////////////
#define MPU6050_ADR 0x68

////////////////////////////////
//         DEFINITION         //
////////////////////////////////
typedef enum {
    MPU6050_GYRO_CFG = 0x1b,
    MPU6050_ACCEL_CFG = 0x1c,
    MPU6050_SMPLRT_DIV = 0x19,
    MPU6050_ACCEL_XOUT_H = 0x3b,
    MPU6050_ACCEL_XOUT_L = 0x3c,
    MPU6050_ACCEL_YOUT_H = 0x3d,
    MPU6050_ACCEL_YOUT_L = 0x3e,
    MPU6050_ACCEL_ZOUT_H = 0x3f,
    MPU6050_ACCEL_ZOUT_L = 0x40,
    MPU6050_GYRO_XOUT_H = 0x43,
    MPU6050_GYRO_XOUT_L = 0x44,
    MPU6050_GYRO_YOUT_H = 0x45,
    MPU6050_GYRO_YOUT_L = 0x46,
    MPU6050_GYRO_ZOUT_H = 0x47,
    MPU6050_GYRO_ZOUT_L = 0x48,
    MPU6050_PWR_MGMT_1 = 0x6b,
    MPU6050_WHO_AM_I = 0x75
} mpu6050_reg_t;

typedef enum {
    MPU6050_OK,
    MPU6050_FAIL
} mpu6050_status_t;

typedef struct {
    hal_i2c_t i2c;
    int16_t ax, ay, az;
    int16_t gx, gy, gz; 
} mpu6050_t;

mpu6050_status_t mpu6050_init(const mpu6050_t* accel);

static int16_t mpu6050_read(mpu6050_reg_t high_reg, mpu6050_reg_t low_reg, const mpu6050_t* accel);

int16_t mpu6050_read_ax(const mpu6050_t* accel);
int16_t mpu6050_read_ay(const mpu6050_t* accel);
int16_t mpu6050_read_az(const mpu6050_t* accel);

int16_t mpu6050_read_gx(const mpu6050_t* accel);
int16_t mpu6050_read_gy(const mpu6050_t* accel);
int16_t mpu6050_read_gz(const mpu6050_t* accel);

void mpu6050_read_accel(mpu6050_t* accel);
void mpu6050_read_gyro(mpu6050_t* accel);
void mpu6050_read_all(mpu6050_t* accel);

////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

mpu6050_status_t mpu6050_init(const mpu6050_t* accel) {
    uint8_t tmp = hal_i2c_r(MPU6050_WHO_AM_I, MPU6050_ADR, accel->i2c);
    if(tmp != 104) return MPU6050_FAIL;

    hal_i2c_w(0, MPU6050_PWR_MGMT_1, MPU6050_ADR, accel->i2c);  // enable accel
    hal_i2c_w(7, MPU6050_SMPLRT_DIV, MPU6050_ADR, accel->i2c);  // 1KHz

    hal_i2c_w(0, MPU6050_GYRO_CFG, MPU6050_ADR, accel->i2c);  // +-250 grad/sec
    hal_i2c_w(0, MPU6050_ACCEL_CFG, MPU6050_ADR, accel->i2c);  // +-2g

    return MPU6050_OK;
}

static int16_t mpu6050_read(mpu6050_reg_t high_reg, mpu6050_reg_t low_reg, const mpu6050_t* accel) {
    uint8_t high = hal_i2c_r(high_reg, MPU6050_ADR, accel->i2c);
    uint8_t low = hal_i2c_r(low_reg, MPU6050_ADR, accel->i2c);

    return (high << 8) | low;
}

int16_t mpu6050_read_ax(const mpu6050_t* accel) {
    return mpu6050_read(MPU6050_ACCEL_XOUT_H, MPU6050_ACCEL_XOUT_L, accel);
}

int16_t mpu6050_read_ay(const mpu6050_t* accel) {
    return mpu6050_read(MPU6050_ACCEL_YOUT_H, MPU6050_ACCEL_YOUT_L, accel);
}

int16_t mpu6050_read_az(const mpu6050_t* accel) {
    return mpu6050_read(MPU6050_ACCEL_ZOUT_H, MPU6050_ACCEL_ZOUT_L, accel);
}

int16_t mpu6050_read_gx(const mpu6050_t* accel) {
    return mpu6050_read(MPU6050_GYRO_XOUT_H, MPU6050_GYRO_XOUT_L, accel);
}

int16_t mpu6050_read_gy(const mpu6050_t* accel) {
    return mpu6050_read(MPU6050_GYRO_YOUT_H, MPU6050_GYRO_YOUT_L, accel);
}

int16_t mpu6050_read_gz(const mpu6050_t* accel) {
    return mpu6050_read(MPU6050_GYRO_ZOUT_H, MPU6050_GYRO_ZOUT_L, accel);
}

void mpu6050_read_accel(mpu6050_t* accel) {
    accel->ax = mpu6050_read_ax(accel);
    accel->ay = mpu6050_read_ay(accel);
    accel->az = mpu6050_read_az(accel);
}

void mpu6050_read_gyro(mpu6050_t* accel) {
    accel->gx = mpu6050_read_gx(accel);
    accel->gy = mpu6050_read_gy(accel);
    accel->gz = mpu6050_read_gz(accel);
}

void mpu6050_read_all(mpu6050_t* accel) {
    mpu6050_read_accel(accel);
    mpu6050_read_gyro(accel);
}

#endif