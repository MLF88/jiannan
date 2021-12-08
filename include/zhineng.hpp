#ifndef ZHINENG_H__
#define ZHINENG_H__

#include <sys/times.h>
#include <opencv2/opencv.hpp>
#include <termios.h>//串口
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include "init_pid.hpp"
#include "motor.hpp"
#include "dp_learning.hpp"

//等待循环；
void mydelay(void);//10ms忙等待；

//打印时间；
void printf_time(clock_t);

//获取矩形中心点；
cv::Point getCenterPoint(cv::Rect rect);

//串口文件打开并初始化设置；return fd;
int my_uart_open(const char *uart_name);
//初始化串口文件；115200,8n1,8位数据，1位停止，没有校验；
//me_uart2:文件描述符，nSpeed:设置波特率(115200,9600),nBits:数据位数（8代表8数据位置）
//nStop：停止位数（1,代表1停止位）;nEvent:数据校验（N:不校验,E：偶校验,O：奇数校验）
int set_attr(int me_uart2, int nSpeed, int nBits, int nStop, char nEvent);

//控制相机俯仰，航向；0x01：俯仰，0x02：航向，0x03:变焦，0x04:拍照；0x05:关机；
//fd: 串口文件；cmd：控制命令（俯仰，航向）；data: pid参数值；camdata: 一组串口数据；int size: 变焦倍数；
//将俯仰和航向数据一起发送；
//发送给app识别目标的标志；
//snd_uart()有7个参数，不用的置为零;
int snd_uart(int me_uart2, unsigned char cmd, int data, int size, unsigned char id, unsigned char cmd2, int data2);

//Pact_Analysis 返回值为5时代表要结束吊舱程序;分析接受到数据；pact:数据包，camera_size:指针，焦距的地址传递。
//return 5,结束吊舱；return 6,关闭TX1电源；
//return 1,结束目标追踪即track_flags设置0,return 2,开启目标追踪即track_flags设置1;
int Pact_Analysis(unsigned char *pact, int *camera_size);

//接受反馈的数据（app发给tx1）; me_uart2: 串口文件; camera_size:指针，焦距的地址传递。
//return 5,结束吊舱；return 6,关闭TX1电源；
//return 1,结束目标追踪即track_flags设置0,return 2,开启目标追踪即track_flags设置1;
int rcv_uart(int me_uart2,int *camsize_area);

//输入当前相机倍数，感兴趣区域坐标；输出在此基础上需要变焦的倍数，以达到已定画幅比例;
int cam_size_comput(int cam_size,cv::Rect trackbox);

//me_fd 串口文件， trackbox 目标框，camsize 相机当前焦距;
//函数完成对绝缘子上下挂点拍照;
int insulator_cam(int me_fd, cv::Rect trackbox, int camsize);

double tracking(cv::Mat &templateImage_bak, cv::Rect &trackBox); //myframe是全局变量；

//同过串口向相机控制板发送pid参数；
//return 2;跳转重新读取神经网络提供的模板和坐标；
//int me_fd: 串口文件，Rect me_trackbox:目标框值,int& cam_resize_count 相机焦距倍数;
//int& center_area 中心点区域; int *img_lim 数组包含目标框长宽的边界;
int send_pid(int me_fd, cv::Rect me_trackbox, int *camsize_area, struct target *gx,int *label_box);//pic;fd;

#endif
