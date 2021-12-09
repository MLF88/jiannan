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
    pthread_track_err=pthread_create(tid+1,NULL,thr_track,(void *)tid);
    if(pthread_track_err)
    {
        fprintf(stderr,"pthread_create-track():%s\n",strerror(pthread_track_err));
        exit(1);
    }
    pthread_learn_err=pthread_create(tid+2,NULL,thr_learn,NULL);
    if(pthread_learn_err)
    {
        fprintf(stderr,"pthread_create-track():%s\n",strerror(pthread_learn_err));
        exit(1);
    }

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

