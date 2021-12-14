#include "../lib/mpu6050.h"


int main() {
    SystemInit();
    SystemCoreClockUpdate();

    hal_use_afio();
    hal_use_gpio(HAL_GPIOB);
    hal_use_i2c(HAL_I2C1);

    hal_i2c_setup(HAL_I2C1);
    hal_i2c_on(HAL_I2C1);

    // read
    mpu6050_t accel = {
        .i2c = HAL_I2C1,
        .ax = 0,
        .ay = 0,
        .az = 0,
        .gx = 0,
        .gy = 0,
        .gz = 0
    };

    mpu6050_status_t res = mpu6050_init(&accel);
    if(res != MPU6050_OK) return 0;

    while(true)
        mpu6050_read_all(&accel);  // look at `accel` struct 

    return 0;
}
