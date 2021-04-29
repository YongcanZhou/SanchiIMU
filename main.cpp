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


//imu
static const uint8_t start[6] = {0xA5,0x5A,0x04,0x01,0x05,0xAA};
static const uint8_t stop[6] = {0xA5,0x5A,0x04,0x02,0x06,0xAA};
static int data_length = 40;
static uint8_t imu_data[40] = { 0 };

int flag=0;
//第一部分代码/
//根据具体的设备修改
const char default_path[] = "/dev/ttyUSB0";



int main(int argc, char *argv[])
{
    int fd;
    char *path;
    uint8_t buf[100] = {0};

    //第二部分代码/

    //若无输入参数则使用默认终端设备
    if (argc > 1)
        path = argv[1];
    else
        path = (char *)default_path;

    //获取串口设备描述符
    printf("This is tty/usart demo.\n");
    fd = open(path, O_RDWR | O_NOCTTY|O_NDELAY);//noctty,not influenced by terminal,
  //  fcntl(fd,F_SETFL,0);
    if (fd < 0)
    {
        printf("Fail to Open %s device\n", path);
        return 0;
    }

    //第三部分代码/
    struct termios opt; //BASIC SETTING
   // struct termios
    //{
   // tcflag_t c_iflag; //输入模式标志
   // tcflag_t c_oflag; //输出模式标志
  //  tcflag_t c_cflag; //控制模式标志
  //  tcflag_t c_lflag; //本地模式标志
  //  cc_t c_line; //line discipline
  //  cc_t c_cc[NCC]; //control characters
  //  }

    //清空串口接收缓冲区
    tcflush(fd, TCIOFLUSH);
    // 获取串口参数opt
    tcgetattr(fd, &opt);


    //设置串口输出波特率
    cfsetospeed(&opt, B115200);
    //设置串口输入波特率
    cfsetispeed(&opt, B115200);
    //设置数据位数 8位数据位
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    //校验位 无校验位
    opt.c_cflag &= ~PARENB;
    opt.c_iflag &= ~INPCK;
    //设置停止位  1位停止位
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag |= CLOCAL | CREAD;
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag &= ~OPOST;
    opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);


    //
    opt.c_cc[VTIME]=11;
    opt.c_cc[VMIN]=0;

    //更新配置
    tcsetattr(fd, TCSANOW, &opt);

    std::cout << "Device ttyUSB0 is set to 115200bps,8N1\n" << std::endl;

    //第四部分代码/
    tcflush(fd,TCIOFLUSH);
    write(fd,stop,6);
    usleep(1000*1000);

    tcflush(fd,TCIFLUSH);
    tcflush(fd,TCOFLUSH);
    tcflush(fd,TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &opt);
    usleep(100*100);
    write(fd,start,6);
    usleep(100*100);
//    fd_set rfds;
    while (1)
    {

        read(fd,buf,sizeof (uint8_t) * data_length);

        std::copy(buf,buf+40,imu_data);

        for(int i = 0;i < data_length;++i)
        {
            if(imu_data[i]==0xa5 && imu_data[i+1]==0x5a && imu_data[i+2]==0x25  )  //&& buf[i+39]==0xaa
            {
                if(i !=0){
                    usleep(1000);
                    read(fd,buf,sizeof (uint8_t)*(i));
                    read(fd, buf, sizeof(uint8_t)*data_length);
                    usleep(1000);

                }

                for(int i = 0;i<data_length;++i)
                {
                    std::cout  << std::hex << +imu_data[i] << "  ";

               }
                std::cout<<std::endl;

                //the scalar of yaw roll and pitch is 0.1degree
                uint16_t yaw = imu_data[3];
                yaw = yaw<<8|imu_data[4];
                int16_t pitch = imu_data[5];
                pitch = pitch<<8|imu_data[6];
                int16_t roll = imu_data[7];
                roll = roll<<8|imu_data[8];
                //after divided by 16384, the scalar of x y z accelaration are g
                int16_t x_acc = imu_data[9];
                x_acc = x_acc<<8|imu_data[10];
                int16_t y_acc = imu_data[11];
                y_acc = y_acc<<8|imu_data[12];
                int16_t z_acc = imu_data[13];
                z_acc = z_acc<<8|imu_data[14];
                //after divided by 32.8 the scalar of x y z gyro data are degreee/second
                int16_t x_gyro = imu_data[15];
                x_gyro = x_gyro<<8| imu_data[16];
                int16_t y_gyro = imu_data[17];
                y_gyro = y_gyro <<8|imu_data[18];
                int16_t z_gyro = imu_data[19];
                z_gyro = z_gyro<<8 |imu_data[20];

                std::cout<<"yaw = "<<0.1*yaw<<" pitch = "<<0.1*pitch<<" roll = "<<0.1*roll;
                std::cout<<std::endl;
                std::cout<<std::setprecision(4)<<"x_acc = "<< x_acc/16384<<" y_acc = "<< y_acc/16384<<" z_acc = "<<z_acc/16384;
                std::cout<<std::endl;
                std::cout<<"x_gyro = "<<x_gyro/32.8<<" y_gyro = "<<y_gyro/32.68<<" z_gyro = "<<z_gyro/32.68;
                std::cout<<std::endl;

            }
        }
        usleep(1000);
    }

    return 0;
}
