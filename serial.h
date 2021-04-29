#ifndef SERIAL_H_
#define SERIAL_H_
#include <stdint.h>
namespace imu{

//imu instruction
static const uint8_t start[6] = {0xA5,0x5A,0x04,0x01,0x05,0xAA};
static const uint8_t stop[6] = {0xA5,0x5A,0x04,0x02,0x06,0xAA};
static int data_length = 40;

//第一部分代码/
//根据具体的设备修改
static char default_path[] = "/dev/ttyUSB0";
static int fd = -1;

static unsigned char prerecstatus;
static unsigned char recstatus;
static unsigned char ccnt = 0;
static unsigned char packerflag = 0;
static uint8_t check = 0;
static unsigned char rxbuf[100];

struct Data{
    double yaw;
    double roll;
    double pitch;

    double x_acc;
    double y_acc;
    double z_acc;

    double x_gyro;
    double y_gyro;
    double z_gyro;
};

static Data data;

Data getCurrentPos();

int initIMU();


int closeIMU();

int startIMU();

}

#endif


