#include<iostream>
#include<iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include"serial.h"

int main(int argc, char *argv[])
{
    using namespace imu;
    initIMU();
    imu::Data data;
    while (1) {
        data = getCurrentPos();

                    std::cout<<"yaw = "<<std::setw(12)<<std::right<<data.yaw<<std::setw(12)<<" pitch = "<<data.pitch<<std::setw(12)<<" roll = "<<data.roll;
                    std::cout<<std::endl;
                    std::cout<<"x_acc = "<<std::setw(12)<< data.x_acc<<std::setw(12)<<" y_acc = "<<std::setw(12)<< data.y_acc<<std::setw(12)<<" z_acc = "<<data.z_acc;
                    std::cout<<std::endl;
                    std::cout<<"x_gyro = "<<std::setw(12)<<data.x_gyro<<std::setw(12)<<" y_gyro = "<<data.y_gyro<<" z_gyro = "<<data.z_gyro;
                    std::cout<<std::endl;
    }

    return 0;
}



