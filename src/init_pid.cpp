#include <stdio.h>
#include "init_pid.hpp"

//pid全局变量参数；
static int err_x = 0,error_last_x = 0, error_last_last_x = 0;//error_x
static float Kp_x=1.5,Ki_x=0.007,Kd_x=0.07;//Pid_x
static float output_p_x,output_i_x,output_d_x, output_pid_x;//output_x

static int err_y = 0,error_last_y = 0, error_last_last_y = 0;//error_y
static float Kp_y=1.5,Ki_y=0.0065,Kd_y=0.065;//Pid_y
static float output_p_y,output_i_y,output_d_y, output_pid_y;//output_y 

static void init_pid_x(float kp_x, float ki_x, float kd_x)
{
    //error-x
    err_x = 0;
    error_last_x = 0;
    error_last_last_x = 0;
    //Pid_x
    Kp_x=kp_x;
    Ki_x=ki_x;
    Kd_x=kd_x;
    //output_x
    output_p_x = 0;
    output_i_x = 0;
    output_d_x = 0; 
    output_pid_x = 0;
}
static void init_pid_y(float kp_y, float ki_y, float kd_y)
{
    //error_y
    err_y = 0;
    error_last_y = 0; 
    error_last_last_y = 0;
    //Pid_y
    Kp_y = kp_y;
    Ki_y = ki_y;
    Kd_y = kd_y;
    //output_y
    output_p_y = 0;
    output_i_y = 0;
    output_d_y = 0;
    output_pid_y = 0;
}
static void init_pid_x_y(float kp_x, float ki_x, float kd_x, float kp_y, float ki_y, float kd_y)
{
    //printf("kp_x=%2.3f,ki_x=%2.3f,kd_x=%2.3f\n",kp_x,ki_x,kd_x);
    //printf("kp_y=%2.3f,ki_y=%2.3f,kd_y=%2.3f\n",kp_y,ki_y,kd_y);
    init_pid_x(kp_x, ki_x, kd_x);
    init_pid_y(kp_y, ki_y, kd_y);
}

void reset_pid_x_y(void)
{
    //error-x
    err_x = 0;
    error_last_x = 0;
    error_last_last_x = 0;
    //output_x
    output_p_x = 0;
    output_i_x = 0;
    output_d_x = 0; 
    output_pid_x = 0;
    
    //error_y
    err_y = 0;
    error_last_y = 0; 
    error_last_last_y = 0;
    //output_y
    output_p_y = 0;
    output_i_y = 0;
    output_d_y = 0;
    output_pid_y = 0;
}

void debug_pid(char cmd, char datah,char datal)
{
    int data_m = ((datah << 8)|datal);
	//printf("********debug_pid data_m=%d**************\n",data_m);
    float data = data_m/10000.0f;
    //printf("*************debug_pid data=%f**********\n", data);

    reset_pid_x_y();

    switch(cmd)
    {
        case 0x40:
            Kp_x = data;
            break;
        case 0x41:
            Ki_x = data;
            break;
        case 0x42:
            Kd_x = data;
            break;
        case 0x43:
            Kp_y = data;
            break;
        case 0x44:
            Ki_y = data;
            break;
        case 0x45:
            Kd_y = data;
            break;
        default:
            //fprintf(stderr,"app scanf pid data is err\n");
            break;

    }

    //printf("Kp_x=%2.3f,Ki_x=%2.3f,Kd_x=%2.3f\n",Kp_x,Ki_x,Kd_x);
    //printf("Kp_y=%2.3f,Ki_y=%2.3f,Kd_y=%2.3f\n",Kp_y,Ki_y,Kd_y);

}

//init_pi_param(int size)函数根据传入的相机焦距重置PID参数，并设置中心区域，没有返回值；
void init_pid_param(int *camsize_area)
{
    int *size_ar = camsize_area;
    //printf("size=%d\n",size_ar[0]);
    switch(size_ar[0])
    {
        case 1:
        {
            init_pid_x_y(1.5, 0.007, 0.07, 1.5, 0.0065, 0.065);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 10;
            break;
        }
        case 2:
        {
            init_pid_x_y(1.5, 0.007, 0.07, 1.4, 0.0065, 0.065);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 10;
            break;
        }
        case 3:
        {
            init_pid_x_y(1.5, 0.0065, 0.065, 1.3, 0.0065, 0.065);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 10;
            break;
        }
        case 4:
        {
            init_pid_x_y(1.2, 0.0065, 0.065, 1.1, 0.0065, 0.065);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 10;
            break;
        }
        case 5:
        {
            init_pid_x_y(1.1, 0.0065, 0.065, 1.1, 0.002, 0.03);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 20;
            break;
        }
        case 6:
        {
            init_pid_x_y(1.0, 0.005, 0.05, 1.1, 0.001, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 25;
            break;
        }
        case 7:
        {
            init_pid_x_y(1.1, 0.004, 0.04, 1.0, 0.004, 0.03);
            size_ar[1] = 25;
            break;
        }
        case 8:
        {
            init_pid_x_y(1.1, 0.002, 0.01, 1.0, 0.002, 0.02);
            size_ar[1] = 25;
            break;
		}
        case 9:
        {
            init_pid_x_y(1.0, 0.002, 0.01, 1.0, 0.002, 0.02);
            //init_pid_x_y(0.9, 0.001, 0.01, 0.8, 0.001, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 25;
            break;
		}
        case 10:
        {
            init_pid_x_y(0.9, 0.002, 0.02, 0.8, 0.001, 0.02);
            //init_pid_x_y(0.9, 0.001, 0.01, 0.7, 0.001, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 25;
            break;
		}
        case 11:
        {
            init_pid_x_y(0.9, 0.001, 0.01, 0.7, 0.001, 0.02);
            //init_pid_x_y(0.8, 0.003, 0.03, 0.7, 0.003, 0.02);
            size_ar[1] = 25;
            break;
		}
        case 12:
        {
            init_pid_x_y(0.8, 0.003, 0.03, 0.7, 0.003, 0.02);
            //init_pid_x_y(0.75, 0.003, 0.03, 0.65, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 26;
            break;
		}
        case 13:
        {
            init_pid_x_y(0.75, 0.003, 0.03, 0.65, 0.003, 0.02);
            //init_pid_x_y(0.7, 0.003, 0.03, 0.6, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 27;
            break;
		}
        case 14:
        {
            init_pid_x_y(0.7, 0.003, 0.03, 0.6, 0.003, 0.02);
            //init_pid_x_y(0.65, 0.003, 0.03, 0.55, 0.003, 0.02);
            size_ar[1] = 27;
            break;
		}
        case 15:
		{
            //init_pid_x_y(0.65, 0.003, 0.03, 0.55, 0.003, 0.02);
            init_pid_x_y(0.6, 0.003, 0.03, 0.5, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 28;
            break;
		}
        case 16:
        {
            //init_pid_x_y(0.6, 0.003, 0.03, 0.5, 0.003, 0.02);
            init_pid_x_y(0.55, 0.003, 0.03, 0.45, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 28;
            break;
		}
        case 17:
        {
            //init_pid_x_y(0.55, 0.003, 0.03, 0.45, 0.003, 0.02);
            init_pid_x_y(0.5, 0.003, 0.03, 0.4, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 26;
            break;
		}
		case 18:
        {
            //init_pid_x_y(0.5, 0.003, 0.03, 0.4, 0.003, 0.02);
            init_pid_x_y(0.45, 0.003, 0.03, 0.35, 0.003,0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 27;
            break;
		}
        case 19:
        {
            //init_pid_x_y(0.45, 0.003, 0.03, 0.35, 0.003,0.02);
            init_pid_x_y(0.4, 0.003, 0.03, 0.3, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 28;
            break;
        }
        case 20:
        {
            //init_pid_x_y(0.4, 0.003, 0.03, 0.3, 0.003, 0.02);
            init_pid_x_y(0.35, 0.003, 0.03, 0.3, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 28;
            break;
        }
        case 21:
        {
            init_pid_x_y(0.35, 0.003, 0.03, 0.3, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 28;
            break;
        }
        case 22:
        {
            init_pid_x_y(0.3, 0.003, 0.03, 0.35, 0.003, 0.02);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 28;
            break;
        }
        case 23:
        {
            init_pid_x_y(0.3, 0.002, 0.1, 0.3, 0.002, 0.2);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 28;
            break;
        }
        case 24:
        {
            init_pid_x_y(0.3, 0.002, 0.1, 0.3, 0.002, 0.1);// kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 33;
            break;
        }
        case 25:
		{
            init_pid_x_y(0.3, 0.0018, 0.07, 0.3, 0.0018, 0.07); // kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 34;
            break;
        }
        case 26:
        {
            init_pid_x_y(0.3, 0.0026, 0.05, 0.25, 0.0026, 0.05); // kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 35;
            break;
        }
        case 27:
        {
            init_pid_x_y(0.2, 0.0026, 0.05, 0.2, 0.0026, 0.05); // kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            //init_pid_x_y(0.3, 0.0026, 0.05, 0.3, 0.0026, 0.05, 30); // kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 36;
            break;
        }
        case 28:
        {
            init_pid_x_y(0.17, 0.0026, 0.05, 0.17, 0.0026, 0.05); // kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 37;
            break;
        }
        case 29:
        {
            init_pid_x_y(0.17, 0.0026, 0.05, 0.17, 0.0026, 0.05); // kp_x,ki_x,kd_x; kp_y,ki_y,kd_y;
            size_ar[1] = 37;
            break;
        }
        default:
            //fprintf(stderr,"Camera size is err\n");
            break;
    }
}

/** x轴方向的pid函数；Pid controler Function  **/
//input： pv 像素实际坐标  sp 期望坐标
//return： pwm;  x轴方向的pid函数；
int pid_controler_x(int pv,int  sp, int center_area)	
{
    if( (pv>=(sp-center_area)) && (pv<=(sp+center_area)) )
        return 1500;

    err_x = sp - pv;   	
    output_p_x += (Kp_x*(err_x - error_last_x));	
    output_i_x += (Ki_x*err_x);

    if(output_i_x>300)   
        output_i_x=300;

    if(output_i_x < -300)     
        output_i_x= -300;

    output_d_x += (Kd_x*(err_x - 2 * error_last_x + error_last_last_x));  //pid	    
    output_pid_x = output_p_x + output_i_x + output_d_x;  
    error_last_last_x = error_last_x;	
    error_last_x = err_x;

    if(output_pid_x<-400)
        return 1100;
    else if (output_pid_x>400)
        return 1900;   
    else  
        return 1500+output_pid_x;
}

/** y轴方向的pid函数； Pid controler Function  **/
//input： pv 像素实际坐标  sp 期望坐标
//return： pwm
int pid_controler_y(int pv,int sp, int center_area)	
{
    if( (pv>=(sp-center_area)) && (pv<=(sp+center_area)) )
	{
		//Kp_y=0,Ki_y=0,Kd_y=0;//Pid_y
	
		return 1500;
	}
    err_y = sp - pv;   	
    output_p_y += (Kp_y*(err_y - error_last_y));	
    output_i_y += (Ki_y*err_y);

    if(output_i_y>300)   
        output_i_y=300;

    if(output_i_y < -300)     
        output_i_y= -300;

    output_d_y += (Kd_y*(err_y - 2 * error_last_y + error_last_last_y));  //pid	    
    output_pid_y = output_p_y + output_i_y + output_d_y;  
	//printf("output_pid_y=%f,output_p_y=%f,output_i_y=%f,output_d_y=%f\n",output_pid_y,output_p_y,output_i_y,output_d_y);
    error_last_last_y = error_last_y;	
    error_last_y = err_y;

    if(output_pid_y<-400)
        return 1100;
    else if (output_pid_y>400)
        return 1900;   
    else  
        return 1500+output_pid_y;
}

