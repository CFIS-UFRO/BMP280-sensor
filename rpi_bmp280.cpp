/*
Based on the BMP280 datasheet and:
- https://github.com/leon-anavi/rpi-examples/blob/master/BMP280/c/BMP280.c
- https://github.com/adafruit/Adafruit_CircuitPython_BMP280/blob/f65303074498e07db883e73c067a8b9e491451a2/adafruit_bmp280.py
Author: Bryan Casanelli - bryancasanelli@gmail.com
*/

#include <unistd.h> //usleep
#include <stdint.h>
#include "rpi_bmp280.h"
#ifndef __arm__
  #include "fake_wiringPiI2C.h"
  #include "fake_wiringPi.h"
#else
  #include <wiringPiI2C.h>
  #include <wiringPi.h>
#endif

/* BMP280 default address */
#define   BMP280_I2CADDR 0x76
#define   BMP280_CHIPID  0xD0

/* BMP280 Registers */
#define   BMP280_DIG_T1 0x88  /* R   Unsigned Calibration data (16 bits) */
#define   BMP280_DIG_T2 0x8A  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_T3 0x8C  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_P1 0x8E  /* R   Unsigned Calibration data (16 bits) */
#define   BMP280_DIG_P2 0x90  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_P3 0x92  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_P4 0x94  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_P5 0x96  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_P6 0x98  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_P7 0x9A  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_P8 0x9C  /* R   Signed Calibration data (16 bits)   */
#define   BMP280_DIG_P9 0x9E  /* R   Signed Calibration data (16 bits)   */

#define   BMP280_CONTROL      0xF4
#define   BMP280_RESET        0xE0
#define   BMP280_CONFIG       0xF5
#define   BMP280_PRESSUREDATA 0xF7
#define   BMP280_TEMPDATA     0xFA

#define IIR_FILTER_DISABLE 0x00
#define IIR_FILTER_X2      0x01
#define IIR_FILTER_X4      0x02
#define IIR_FILTER_X8      0x03
#define IIR_FILTER_X16     0x04

#define OVERSCAN_DISABLE 0x00
#define OVERSCAN_X1      0x01
#define OVERSCAN_X2      0x02
#define OVERSCAN_X4      0x03
#define OVERSCAN_X8      0x04
#define OVERSCAN_X16     0x05

#define MODE_SLEEP  0x00
#define MODE_FORCE  0x01
#define MODE_NORMAL 0x03

#define STANDBY_0_5  0x00  // 0.5ms
#define STANDBY_62_5 0x01  // 62.5ms
#define STANDBY_125  0x02  // 125ms
#define STANDBY_250  0x03  // 250ms
#define STANDBY_500  0x04  // 500ms
#define STANDBY_1000 0x05  // 1000ms
#define STANDBY_2000 0x06  // 2000ms
#define STANDBY_4000 0x07  // 4000ms

rpi_bmp280::rpi_bmp280(){
  wiringPiSetup();
  this->overscan_vector   = {OVERSCAN_DISABLE,OVERSCAN_X1,OVERSCAN_X2,OVERSCAN_X4,OVERSCAN_X8,OVERSCAN_X16};
  this->mode_vector       = {MODE_SLEEP,MODE_FORCE,MODE_NORMAL};
  this->standby_vector    = {STANDBY_0_5,STANDBY_62_5,STANDBY_125,STANDBY_250,STANDBY_500,STANDBY_1000,STANDBY_2000,STANDBY_4000};
  this->iir_filter_vector = {IIR_FILTER_DISABLE,IIR_FILTER_X2,IIR_FILTER_X4,IIR_FILTER_X8,IIR_FILTER_X16};
}

int rpi_bmp280::init(int id){
  this->fd = wiringPiI2CSetup(id);
  if (this->fd == -1){return -1;} // Error
  if (0x58 != wiringPiI2CReadReg8(this->fd, BMP280_CHIPID)){return -1;} // Error
  this->reset();
  this->load_calibration();
  this->config(1,1,2,0,0);
  return 0;
}

void rpi_bmp280::reset(){
  wiringPiI2CWriteReg8(this->fd, BMP280_RESET, 0xB6);
  usleep(5000);
}

void rpi_bmp280::config(int p_overscan, int t_overscan, int mode, int standby, int filter){
  this->overscan_temperature = this->overscan_vector.at(p_overscan);
  this->overscan_pressure    = this->overscan_vector.at(t_overscan);
  this->mode                 = this->mode_vector.at(mode);
  this->standby              = this->standby_vector.at(standby);
  this->iir_filter           = this->iir_filter_vector.at(filter);
  this->write_control();
  this->write_config();
}

void rpi_bmp280::write_control(){
  uint8_t to_write = 0;
  to_write += this->overscan_temperature << 5;
  to_write += this->overscan_pressure << 2;
  to_write += this->mode;
  wiringPiI2CWriteReg8(this->fd, BMP280_CONTROL, to_write);
  usleep(5000);
}

void rpi_bmp280::write_config(){
  //Writes to the config register may be ignored while in Normal mode
  bool is_normal = false;
  if (this->mode == MODE_NORMAL){
    is_normal  = true;
    this->mode = MODE_SLEEP;
    this->write_control();
  }

  uint8_t config = 0;
  config += this->standby << 5;
  config += this->iir_filter << 2;
  wiringPiI2CWriteReg8(this->fd, BMP280_CONFIG, config);
  usleep(5000);

  if (is_normal){
    this->mode = MODE_NORMAL;
    this->write_control();
  }
}

void rpi_bmp280::load_datasheet_calibration(){
  this->cal_t1 = 27504;
  this->cal_t2 = 26435;
  this->cal_t3 = -1000;
  this->cal_p1 = 36477;
  this->cal_p2 = -10685;
  this->cal_p3 = 3024;
  this->cal_p4 = 2855;
  this->cal_p5 = 140;
  this->cal_p6 = -7;
  this->cal_p7 = 15500;
  this->cal_p8 = -14500;
  this->cal_p9 = 6000;
}

void rpi_bmp280::load_calibration(){
  this->cal_t1 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_T1);
  this->cal_t2 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_T2);
  this->cal_t3 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_T3);
  this->cal_p1 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P1);
  this->cal_p2 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P2);
  this->cal_p3 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P3);
  this->cal_p4 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P4);
  this->cal_p5 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P5);
  this->cal_p6 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P6);
  this->cal_p7 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P7);
  this->cal_p8 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P8);
  this->cal_p9 = wiringPiI2CReadReg16(this->fd, BMP280_DIG_P9);
}

uint32_t rpi_bmp280::read_raw(int reg){
  uint32_t raw;
  raw = wiringPiI2CReadReg8(this->fd, reg);
  raw <<= 8;
  raw += wiringPiI2CReadReg8(this->fd, reg+1);
  raw <<= 8;
  raw += wiringPiI2CReadReg8(this->fd, reg+2);
  raw >>= 4;
  return raw;
}

int32_t rpi_bmp280::compensate_temp(int32_t raw_temp){
  int32_t t1 = (((raw_temp >> 3) - ((int32_t)(this->cal_t1) << 1)) * ((int32_t)(this->cal_t2))) >> 11;
  int32_t t2 = (((((raw_temp >> 4) - ((int32_t)(this->cal_t1))) * ((raw_temp >> 4) - ((int32_t)(this->cal_t1)))) >> 12) * ((int32_t)(this->cal_t3))) >> 14;
  return t1 + t2;
}

// °C
double rpi_bmp280::read_temperature(){
  uint32_t raw_temp = this->read_raw(BMP280_TEMPDATA);
  int32_t compensated_temp = this->compensate_temp(raw_temp);
  return (double)((compensated_temp * 5 + 128) >> 8) / 100.0;
}

// Pa
double rpi_bmp280::read_pressure(){
  uint32_t raw_temp        = this->read_raw(BMP280_TEMPDATA);
  int64_t compensated_temp = (int64_t)(compensate_temp(raw_temp));
  int64_t raw_pressure     = (int64_t)(read_raw(BMP280_PRESSUREDATA));

  int64_t p1, p2;
  p1  = compensated_temp - 128000;
  p2  = p1 * p1 * (int64_t)(this->cal_p6);
  p2 += ((p1*(int64_t)(this->cal_p5)) << 17);
  p2 += ((int64_t)(this->cal_p4) << 35);
  p1 = ((p1 * p1 * (int64_t)(this->cal_p3)) >> 8) + ((p1 * (int64_t)(this->cal_p2)) << 12);
  p1 = (((int64_t)(1) << 47) + p1) * ((int64_t)(this->cal_p1)) >> 33;

  if (0 == p1){return 0;} // Avoid exception caused by division by zero

  int64_t p = 1048576 - raw_pressure;
  p  = (((p << 31) - p2) * 3125) / p1;
  p1 = ((int64_t)(this->cal_p9) * (p >> 13) * (p >> 13)) >> 25;
  p2 = ((int64_t)(this->cal_p8) * p) >> 19;
  p  = ((p + p1 + p2) >> 8) + (((int64_t)(this->cal_p7)) << 4);

  return (double)(p) / 256.0;
}
