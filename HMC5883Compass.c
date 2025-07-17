#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "math.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define addr 0x0D

float get_heading(int16_t x, int16_t y);
void calibrate_compass(int16_t x, int16_t y, int16_t* x_corrected, int16_t* y_corrected);

void compass_init(void) 
{
    // Wait for compass to boot up fr
    sleep_ms(2000);
    uint8_t config[2];
    config[0] = 0x09;
    config[1] = 0b00011101;
    i2c_write_blocking(I2C_PORT, addr, config, 2, true);

    // Continuous mode
    uint8_t data1[2];
    data1[0] = 0x0B;
    data1[1] = 0x01;
    i2c_write_blocking(I2C_PORT, addr, data1, 2, true);

    uint8_t data2[2];
    data2[0] = 0x09;
    data2[1] = 0x1D;

    i2c_write_blocking(I2C_PORT, addr, data2, 2, false);
}

float get_heading(int16_t x, int16_t y) 
{
    // atan2f returns radians between -pi and +pi
    float heading_rad = atan2f((float)y, (float)x);

    // Convert radians to degrees
    float heading_deg = heading_rad * (180.0f / M_PI);

    // Normalize to 0–360 degrees
    if (heading_deg < 0) {
        heading_deg += 360.0f;
    }
    return heading_deg;
}

void calibrate_compass(int16_t x, int16_t y, int16_t* x_corrected, int16_t* y_corrected)
{
    int16_t x_min = -113, x_max = 1210;
    int16_t y_min = -1872, y_max = 23;
    //int16_t z_min = -485, z_max = 358;

    int16_t x_offset = (x_min + x_max) / 2;
    int16_t y_offset = (y_min + y_max) / 2;

    *x_corrected = x - x_offset;
    *y_corrected = y - y_offset;
}

float read_compass(void) 
{ 
    // Measurement
    uint8_t status_reg = 0x06;
    uint8_t status = 0;
    i2c_write_blocking(I2C_PORT, addr, &status_reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, &status, 1, false);

    if ((status & 0x01) == 1) 
    {
        uint8_t data_reg = 0x00;
        uint8_t dataComp[6];

        i2c_write_blocking(I2C_PORT, addr, &data_reg, 1, true);
        i2c_read_blocking(I2C_PORT, addr, dataComp, 6, false);
        int16_t x = (dataComp[1] << 8) | dataComp[0];
        int16_t y = (dataComp[3] << 8) | dataComp[2];
        //int16_t z = (dataComp[5] << 8) | dataComp[4];

        printf("%d %d\n", x, y);
        int16_t x_corrected, y_corrected;
        calibrate_compass(x, y, &x_corrected, &y_corrected);
        return get_heading(x_corrected, y_corrected);
    }
}


int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    //initialize the compass uwu
    compass_init();
    //read the compass uwu
    while(1) 
    {
        float car_heading = read_compass();
        printf("%f\n", car_heading);
    }
}
