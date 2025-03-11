#include <stdio.h>
#include "pico/stdlib.h"
#include "RP2040.h"
#include "hardware/i2c.h"

void Mpu6050_Init(int16_t direccion_mpu)
{
    uint8_t buf[2];
    buf[0] = 0x6B; // registro de administracion de energia
    buf[1] = 0x00; // establezco un 0 para despertar al MPU6050
    i2c_write_blocking(i2c0, direccion_mpu, buf, 2, false);
}

void Mpu6050_comm(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t direccion_mpu)
{
    uint8_t buffer[8];

    buffer[0] = 0x3B;

    i2c_write_blocking(i2c0, direccion_mpu, buffer, 1, true);  // envio el registro a leer
    i2c_read_blocking(i2c0, direccion_mpu, buffer, 6, false);  // leo el mpu6050

    *accel_x = (buffer[0] << 8) | buffer[1]; // guardo la informacion del eje x
    *accel_y = (buffer[2] << 8) | buffer[3]; // guardo la informacion del eje y
    *accel_z = (buffer[4] << 8) | buffer[5]; // guardo la informacion del eje z
}

void Mpu6050_conf(int16_t direccion_mpu)
{
    uint8_t buffer = 0x1C;
    i2c_write_blocking(i2c0, direccion_mpu, &buffer, 1, true); // envio el registro a escrbir
    buffer = 0b00000000;
    i2c_write_blocking(i2c0, direccion_mpu, &buffer, 1, false); // seteo el full scale range en 2g
}