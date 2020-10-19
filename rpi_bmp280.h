#ifndef RPI_BMP280_h
#define RPI_BMP280_h

#include <stdint.h>
#include <vector>

class rpi_bmp280 {
  public:
    rpi_bmp280();
    int init(int id);
    void config(int p_overscan, int t_overscan, int mode, int standby, int filter);
    double read_temperature();
    double read_pressure();
  private:
    void reset();
    void write_control();
    void write_config();
    void load_datasheet_calibration();
    void load_calibration();
    uint32_t read_raw(int reg);
    int32_t compensate_temp(int32_t raw_temp);
    int fd;
    uint16_t cal_t1;
    int16_t cal_t2;
    int16_t cal_t3;
    uint16_t cal_p1;
    int16_t cal_p2;
    int16_t cal_p3;
    int16_t cal_p4;
    int16_t cal_p5;
    int16_t cal_p6;
    int16_t cal_p7;
    int16_t cal_p8;
    int16_t cal_p9;
    uint8_t overscan_temperature;
    uint8_t overscan_pressure;
    uint8_t mode;
    uint8_t standby;
    uint8_t iir_filter;
    std::vector<uint8_t> overscan_vector;
    std::vector<uint8_t> mode_vector;
    std::vector<uint8_t> standby_vector;
    std::vector<uint8_t> iir_filter_vector;
};

#endif
