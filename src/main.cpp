#include <stdlib.h>
#include <sys/types.h> //基本系统数据类型

#include "camera.hpp" //相机;
#include "zhineng.hpp"
#include "dp_argu.hpp"

using namespace cv;
using namespace std;

 
int track_flags=0;//开启追踪算法并控制相机的标记; 1开启, 0关闭;
int read_dp_flags;//读取神经网络模型的标记变量，0读取，1不读取;

//myframe是从相机取出的视频帧并经过缩放的视频帧, 在程序中三个线程中共享;
Mat myframe;
pthread_mutex_t mut_myframe, mut_my_targ;//互斥量；
//此变量多线程共用的全局变量，发送给app端模型数量的标记, 为减少发送次数, 设一个标记变量，每帧加1，大于3帧时才发送给app数据;
int sendapp_model_flags=1; 
int pthr_flags=0; //深度学习线程结束标记;

struct target gx[CLASS_NUM];

char save_pic[64] = SAVE_PIC;

void cleanup_func(void *p)
{
    char *ch;
    ch = (char *)p;
    pthread_mutex_unlock(&mut_myframe);
    for(int i=1; i<CLASS_NUM; i++)
        pthread_mutex_unlock(&(gx[i].mut_my_gray));

	puts(ch);
}
void cleanup_func_camera(void *p)
{
    int cam_fd= *((int *)p);
    close(cam_fd);
}

void save_name(char *name)
{
	time_t t;		
	struct tm *tmp;
	char time_save[64];
	time(&t);
	tmp = localtime(&t);

	if(strftime(time_save, 64,"%d%H%M%S",tmp) == 0)
		printf("buffer length 64 is too small\n");
	else 
		sprintf(name,"%s%s",name,time_save);

}

//从相机中读取视频帧;
void *thr_read(void *p)
{
    char webcam_name[15] = CAM_MY;//相机;
    int *cam_fd;
    int h = IMAGEHEIGHT*3/2,dev_ret=0;
    Mat bgrImg,yuvImg1;
    yuvImg1.create(h,IMAGEWIDTH,CV_8UC1);
    myframe.create(IMGHEI,IMGWID,CV_8UC3);//输入;

    //打开相机
    dev_ret = init_device(webcam_name);//初始化摄像头;
    if(dev_ret < 0)
        fprintf(stderr,"open camera dev is failed\n");
    cam_fd = &dev_ret;
    pthread_cleanup_push(cleanup_func_camera,cam_fd);
         
    while(1)
    {
        //从摄像头读取视频;
        do
        {
            bgrImg = read_frame(yuvImg1, bgrImg);//read frame;

        }while(!bgrImg.empty());

        pthread_testcancel();//设置取消点;      
        pthread_mutex_lock(&mut_myframe);//加锁;
        resize(bgrImg,myframe,Size(IMGWID,IMGHEI)); //更新共享视频帧，resize frame;
        pthread_mutex_unlock(&mut_myframe);//解锁;
        pthread_testcancel();//设置取消点;
    }
    pthread_cleanup_pop(1);
    pthread_exit(NULL);
}

//从深度学习算法读取目标框;
int read_dp_unlock(Mat &me_roi, Rect &me_trackbox, int *label_box) 
{
    int i = *label_box;
	
	//判断共享全局变量为空，即被刷新还没有出新的结果;
    if(gx[i].my_gray.empty() )  
    {
        //printf("my_gray is empty \n");
        me_roi.release();
        usleep(35000); 	   
        return -2;//目标为空;  函数返回
    }
    //my_gray(my_learnbox).copyTo(me_roi);
    gx[i].my_gray.copyTo(me_roi);//从全局变量中读取目标截图;
    if(me_roi.empty() )//如果从共享存储空间中读取的目标为空;
    {
        usleep(30000);
        return -2;//目标为空; 函数返回;
    }
    else
        me_trackbox=gx[i].my_learnbox;  //读取目标框坐标值;   

    return 0;
}

//控制相机线程；初始化串口，通过串口发送控制相机的指令，并接受app发送过来的指令;
//读取深度学习的目标框;同时读取实时的视频帧;
void *thr_track(void *p)
{
    const char *ch="cleanup_thr_track";
    pthread_t *pthr_id=(pthread_t *)p;

    pthread_cleanup_push(cleanup_func,(void *)ch);
    int fd_uart; //串口文件;
    int rcv_return_value;

    int camsize_area[2];     //img_limit[2]目标框宽, 长的界限;
    
    camsize_area[0] = 1; //相机当前的焦距;
    camsize_area[1] = 20; //中心区域;

    //在track线程中接受共享全局变量my_learnbox,作为函数tracking()的参数;
    Rect my_trackbox;
    //在追踪匹配线程中从深度学习线程截取为my_roi感兴趣区域,作为追踪匹配的感兴趣区域;
    Mat my_roi; 
    //创建并初始化串口;如果失败则召回其他线程,并退出本线程;
    fd_uart=my_uart_open(UART2);
    if(fd_uart < 0)
    {
        pthread_cancel(pthr_id[0]);
        pthread_cancel(pthr_id[2]);
        pthread_exit(NULL);
    }
    // 设置串口属性参数,115200,8n1,8位数据位，1位停止位，无校验;
    //me_uart2:文件描述符，nSpeed:设置波特率(115200,9600),nBits:数据位数（8代表8数据位置）
    //nStop：停止位数（1,代表1停止位）;nEvent:数据校验（N:不校验,E：偶校验,O：奇数校验）   
    if(set_attr(fd_uart, 115200, 8, 1, N) < 0)
    {
        close(fd_uart);
        pthread_cancel(pthr_id[0]);
        pthread_cancel(pthr_id[2]);
        pthread_exit(NULL);
    }
    

    gx[1].score_limit = 2*MY_SCORE_TOWE;
    gx[1].image_lim[0] = IMGWID_LIMIT_TOWE;
    gx[1].image_lim[1] = IMGHEI_LIMIT_TOWE;

    gx[2].score_limit = 2*MY_SCORE_HANG;
    gx[2].image_lim[0] = IMGWID_LIMIT_HANG;
    gx[2].image_lim[1] = IMGHEI_LIMIT_HANG;

    gx[3].score_limit = 2*MY_SCORE_INSU;
    gx[3].image_lim[0] = IMGWID_LIMIT_INSU;
    gx[3].image_lim[1] = IMGHEI_LIMIT_INSU;

    gx[4].score_limit = 2*MY_SCORE_NEST;
    gx[4].image_lim[0] = IMGWID_LIMIT_NEST;
    gx[4].image_lim[1] = IMGHEI_LIMIT_NEST;
    
    for(int k=1; k<CLASS_NUM; k++)
    {
        gx[k].img_limit[0] = (int)IMGHEI*gx[k].image_lim[0];
        gx[k].img_limit[1] = (int)IMGHEI*gx[k].image_lim[1];
    }
    
    int label_box_name = 3;
    int *label_box = &label_box_name;
    //int track_frame=0;
    while(1)
    {
        //查询模型变量标记, 并回复给app目标数量的串口指令;
        if(sendapp_model_flags>5)
        {
            pthread_mutex_lock(&mut_my_targ);//加锁;
            sendapp_model_flags = 0;
            if(gx[*label_box].my_box_score >(gx[*label_box].score_limit))
            {    
                gx[*label_box].my_box_score = 0.0;
				//printf("gx[%d].my_box_score=%f\n",*label_box,gx[*label_box].my_box_score);
                pthread_mutex_unlock(&mut_my_targ);//解锁;
                //printf("send data to app 1 targ\n");
                //发送给app的指令;；caffe-ssd识别到一个目标，作为提示用户;
                snd_uart(fd_uart, 0x21, 1, 0, 0, 0, 0);
                usleep(10000);
            }
            else
            {
                gx[*label_box].my_box_score = 0.0;
                pthread_mutex_unlock(&mut_my_targ);//解锁;
                //printf("send data to app none targ\n");
                snd_uart(fd_uart, 0x21, 0, 0, 0, 0, 0);//没有识别到目标;发送个app;
                usleep(10000);
            }
        }
        // 接受串口指令
        rcv_return_value = rcv_uart(fd_uart, camsize_area);// 接受串口指令
        switch(rcv_return_value)
        {
            case 1:     
                {   //接受到将t_flags设置为0的指令;
                    read_dp_flags = 0;
                    track_flags=0;
                    usleep(20000);
                    snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500);
                    usleep(5000);
                    reset_pid_x_y();           
                }
                continue;
            case 2:
                {   
                    //接受到将t_flags设置为1的指令;
                    track_flags=1;
                    *label_box = 3;
                    reset_pid_x_y(); 
                    if(read_dp_flags == 0)//读取模型的标记，只读一次，提供模型;
                    {      
                        snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500); //发送相机停止俯仰航向指令;
                        pthread_mutex_lock(&gx[i].mut_my_gray);//加锁;
                        ret_read_dp = read_dp_unlock(my_roi, my_trackbox, label_box);
                        pthread_mutex_unlock(&(gx[i].mut_my_gray));//解锁;
                        if(ret_read_dp < 0) //返回-2为读取模型失败;
                            continue;   
                        //读取模型成功;
                        read_dp_flags == 1; //0为读取，1为不读取;
                    }
                    //开始接手控制相机;
                    //if(*label_box==3)
                    //track_frame++;
                    if(tracking(my_roi, my_trackbox) < 0.10)
                    {
                        //tracking返回值小0.85，说明目标跟丢了或者有遮挡物体;
                        //应立即停止相机的俯仰和航向， 由前一帧提供模型或者由神经网络提供模型和坐标;
                        snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500); //发送相机停止俯仰航向指令;
                        sleep(2);//给深度学习时间，相机控制指令叠加延时性;
                        reset_pid_x_y(); //PID系统参数清零，重新锁定目标;
                        read_dp_flags == 0;//0为读取，1为不读取;
                        continue;//重新由神经网络提供模型和坐标;
                    }
                    //从模板匹配返回中获得图像中目标实际的位置;
                    //发送向相机控制板发送指令
                    send_pid_rev = send_pid(fd_uart, my_trackbox, camsize_area, &gx[*label_box], label_box);
                    switch (send_pid_rev)
                    { //发送向相机控制板发送指令,return 2,相机变焦了;跳转重新读取神经网络提供的模板和坐标;
                        case 2://拍照，变焦。
                            read_dp_flags == 0; //0为读取，1为不读取;
                            continue;                      //跳转重新读取神经网络提供的模板和坐标;
                        case 3:
                        {
                            track_flags=0;//结束本次动态追踪;
                            read_dp_flags=0;
                            for(int k=0; k < 5; k++)
				            {
					            snd_uart(me_fd, 0x04, 1, 0, 0, 0, 0);//拍照完成标记。
					            usleep(100000);
				            }
                        }
                        continue;
                        
                        default:
                            continue;
                    }

                    //cv::Point point=getCenterPoint(my_trackbox);
                    //printf("point.x=%d,point.y=%d\n", point.x, point.y);
                }
                continue;
            case 5:
                {    //接受到结束吊舱的指令;
                    //printf("rcv data from app close process\n");
                    sleep(1);
                    pthread_cancel(pthr_id[0]); //read;

                    printf("close fd_uart\n");
                    snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500);//在结束线程前发送相机动作停止指令;
                    sleep(1);
                    close(fd_uart);//关闭串口文件;

                    pthread_exit((void *)1);//结束吊舱程序1;

                }
                break;
            case 6:
                {//接受到关闭TX1电源的指令;
                    printf("rcv data from app close TX1\n");
                    pthread_cancel(pthr_id[0]);
                    pthread_cancel(pthr_id[2]);
        
                    printf("close fd_uart\n");
                    usleep(20000);
                    snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500);//在结束线程前发送相机动作停止指令;
                    close(fd_uart);//关闭串口文件;
        
                    printf("thr_track pthread_exit\n");
                    pthread_exit((void *)2);//关机2;    
                }
                break;
            default:
                {
                    /******to do something*****/
                }    
                break;
        }  
    }
    pthread_cleanup_pop(1);
    pthread_exit(NULL);
}

//运行深度学习网络的线程;
void *thr_learn(void *p)
{
    const char *ch="cleanup_thr_learn";
    pthread_cleanup_push(cleanup_func,(void *)ch);

    //pthread_cleanup_pop(1);
    test(); //深度学习算法;
    pthread_cleanup_pop(1);
    pthread_exit(NULL);
}

int main(int argc, char** argv) 
{
    pthread_t tid[THRNUM];
    int pthread_read_err,pthread_track_err,pthread_learn_err;
    void *tret;
    
    save_name(save_pic);

    //互斥量赋值;
    pthread_mutex_init(&mut_myframe,NULL);
    for(int k=1; k<CLASS_NUM; k++)
    {
        pthread_mutex_init(&(gx[k].mut_my_gray),NULL);
    }
    pthread_mutex_init(&mut_my_targ,NULL);

    //创建线程；两个线程间用一个互斥量锁一个myframe;
    pthread_read_err=pthread_create(tid,NULL,thr_read,NULL);
    if(pthread_read_err)
    {
        fprintf(stderr,"pthread_create-read():%s\n",strerror(pthread_read_err));
        exit(1);
    }
#if 1
    pthread_track_err=pthread_create(tid+1,NULL,thr_track,(void *)tid);
    if(pthread_track_err)
    {
        fprintf(stderr,"pthread_create-track():%s\n",strerror(pthread_track_err));
        exit(1);
    }
#endif
#if  1
    pthread_learn_err=pthread_create(tid+2,NULL,thr_learn,NULL);
    if(pthread_learn_err)
    {
        fprintf(stderr,"pthread_create-track():%s\n",strerror(pthread_learn_err));
        exit(1);
    }
#endif
    //给readframe,track 线程回收资源;
    pthread_join(tid[0],NULL);
    pthread_join(tid[1],&tret);
    pthread_join(tid[2],NULL);
    sleep(1);
    printf("main func is close\n");
    pthread_mutex_destroy(&mut_myframe);
    for(int j=1; j<CLASS_NUM; j++)
    {
        pthread_mutex_destroy(&(gx[j].mut_my_gray));
    }
    pthread_mutex_destroy(&mut_my_targ);
	 
    printf("pthread track return value=%ld\n",(long)tret );
	
    if ((long)tret == 1) //结束吊舱;
        exit(1);
    else if((long)tret == 2) //关机;
        exit(2);

    exit(0);
    
}

