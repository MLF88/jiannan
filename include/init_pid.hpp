#ifndef INIT_PID__H__
#define INIT_PID__H__

//#define IMGWID_LIMIT 504  //设定目标框的边界; 465
//#define IMGHEI_LIMIT 504  //   465

#define IMGWID_LIMIT_TOWE 0.6  //设定杆塔目标框的边界; 0.7,0.6,0.5
#define IMGHEI_LIMIT_TOWE 0.6

#define IMGWID_LIMIT_INSU 0.5  //设定绝缘子目标框的边界; 0.7,0.6,0.5
#define IMGHEI_LIMIT_INSU 0.5   

#define IMGWID_LIMIT_HANG 0.4  //设定下挂点目标框的边界; 0.7,0.6,0.5
#define IMGHEI_LIMIT_HANG 0.4   

#define IMGWID_LIMIT_NEST 0.2 //设定下鸟巢目标框的边界; 0.7,0.6,0.5
#define IMGHEI_LIMIT_NEST 0.2

#define IMGWID 960  //用于计算的目标长宽;
#define IMGHEI 720

//#define TRACKBOX_AREA  146400  //目标框面积的边界;
#define TRACKBOX_AREA  196400  //目标框面积的边界;
#define CENTER_RANGE  25 //目标框中心点范围;
#define THRNUM 3         //线程数量;
#define UART2 "/dev/ttyTHS1" //串口;

//将pid控制的中间变量清零;
void reset_pid_x_y(void);

void debug_pid(char cmd, char datah,char datal);

//init_pi_param(int size)函数根据传入的相机焦距重置PID参数，没有返回值;
void init_pid_param(int *camsize_area);

/** x轴方向的pid函数；Pid controler Function  **/
//input： pv 像素实际坐标  sp 期望坐标
//return： pwm;  x轴方向的pid函数;
int pid_controler_x(int pv,int  sp, int center_area);

/** y轴方向的pid函数； Pid controler Function  **/
//input： pv 像素实际坐标  sp 期望坐标
//return： pwm
int pid_controler_y(int pv,int sp, int center_area);

#endif
