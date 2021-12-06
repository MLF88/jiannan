/*只能被main.cpp包含*/
#ifndef DP_LEARNING__H__
#define DP_LEARNING__H__

#include <pthread.h> //pthread_testcancel();//设置取消点;


#define  SAVE_FRAME_NUM 2 //抽帧的频率

#define MY_SCORE_TOWE 0.3
#define MY_SCORE_HANG 0.5
#define MY_SCORE_INSU 0.25
#define MY_SCORE_NEST 0.20
#define CLASS_NUM   5

#define MANUCAM 7  //手动变焦倍数;

struct target
{
    const char *name;    //目标框名字;
    //在深度学习中输出结果中感兴趣区域的目标框;
    cv::Rect my_learnbox;  //全局变量;
    //my_gray,是深度学习检测输出的视频帧,在learning和track进程中共享视频帧;
    cv::Mat my_gray; // 全局变量;
    //目标框得分连续累计;
	float my_box_score;//每一帧距离中心点最近目标框得分连续累计;
    float score_limit; 
    float image_lim[2]; //画框的比例;
    int img_limit[2];   
    //互斥量;
    pthread_mutex_t mut_my_gray;   
};
void saveframe(cv::Mat img, int frame_index,char *savename, char *path);
int test(void) ;

#endif
