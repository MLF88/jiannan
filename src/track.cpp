#include <stdio.h>

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
//当从串口获取变焦信息时更新当前焦距
int cam_size_update(unsigned char *uart_rcv_buf; int *camsize_area)
{
    int pwm_zoom;
    pwm_zoom = uart_rcv_buf[3] * 256 + uart_rcv_buf[4];  
    int variation=0;
    switch(pwm_zoom)
    {
        case 1200:    //缩小焦距;
            variation = -uart_rcv_buf[5];
            break;
        case 1800:   //放大焦距;
            variation = uart_rcv_buf[5];
            break;
        default:
            fprintf(stderr,"parameter is erro\n");
            return -1;
    }
    //检查焦距是否越界非法;
    camsize_area[0] += variation;
    if(camsize_area[0] < 0)
        camsize_area[0] = 1;
    else if(camsize_area[0] > 30)
        camsize_area[0] = 30;
    
    init_pid_param(camsize_area);
    return camsize_area[0];//返回更新的当前焦距;
}

//查询模型变量标记, 并回复给app目标数量的串口指令;
int queryNumModelUlock(void)
{
    if(sendapp_model_flags>5)
    {
        if(gx[*label_box].my_box_score >(gx[*label_box].score_limit))     
            return 1; 
        
        else
            return 0;
        
        sendapp_model_flags = 0;
        gx[*label_box].my_box_score = 0.0;
    }

    return 0;
}

//销毁线程资源
void *thr_track_destroy(pthread_t *pthr_id,inf fd_uart);
{
    pthread_cancel(pthr_id[0]); //read;
    pthread_cancel(pthr_id[2]);
    sleep(1);
    close(fd_uart);//关闭串口文件;
}

//控制相机线程；初始化串口，通过串口发送控制相机的指令，并接受app发送过来的指令;
//读取深度学习的目标框;同时读取实时的视频帧;
void *thr_track(void *p)
{
    const char *ch="cleanup_thr_track";
    pthread_t *pthr_id=(pthread_t *)p;

    int fd_uart, rcv_return_value, camsize_area[2]; //串口文件;img_limit[2]目标框宽, 长的界限;
    //在track线程中接受共享全局变量my_learnbox,作为函数tracking()的参数;
    Rect my_trackbox;
    //在追踪匹配线程中从深度学习线程截取为my_roi感兴趣区域,作为追踪匹配的感兴趣区域;
    Mat my_roi; 
    camsize_area[0] = 1; //相机当前的焦距;
    camsize_area[1] = 20; //中心区域;

    pthread_cleanup_push(cleanup_func,(void *)ch);
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
        thr_track_destroy(pthr_id,inf fd_uart);
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
    unsigned char uart_rcv_buf[11]='0';
    //int track_frame=0;
    while(1)
    {
        //查询模型变量标记, 并回复给app目标数量的串口指令,作为提示用户;
        pthread_mutex_lock(&mut_my_targ);//加锁;
        unsigned char byModelNum = queryNumModelUlock(void);
        pthread_mutex_unlock(&mut_my_targ);//解锁;
        snd_uart(fd_uart, 0x21, byModelNum, 0, 0, 0, 0);//发送给app的指令;caffe-ssd识别到一个目标;
        usleep(10000);
        // 接受串口指令
        rcv_return_value = rcv_uart(fd_uart, uart_rcv_buf);// 接受串口指令
        switch(rcv_return_value)
        {
            case 2:     
                {   //接受到将t_flags设置为0的指令;
                    read_dp_flags = 0;
                    track_flags=0;
                    usleep(20000);
                    snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500);
                    usleep(5000);
                    reset_pid_x_y();           
                }
                continue;
            case 1:
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
                    if(tracking(my_roi, my_trackbox) < 0.10)
                    {
                        //tracking返回值小0.85，说明目标跟丢了或者有遮挡物体;应立即停止相机的俯仰和航向
                        snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500); //发送相机停止俯仰航向指令;
                        sleep(2);//给深度学习时间，相机控制指令叠加延时性;
                        reset_pid_x_y(); //PID系统参数清零，重新锁定目标;
                        read_dp_flags == 0;//0为读取，1为不读取;
                        continue;//重新由神经网络提供模型和坐标;
                    }
                    //发送向相机控制板发送指令,从模板匹配返回中获得图像中目标实际的位置;
                    send_pid_rev = send_pid(fd_uart, my_trackbox, camsize_area, &gx[*label_box], label_box);
                    switch (send_pid_rev)
                    { //发送向相机控制板发送指令,return 2,相机变焦了;跳转重新读取神经网络提供的模板和坐标;
                        case 2://拍照，变焦。
                            read_dp_flags == 0; //0为读取，1为不读取;
                            continue;                      //跳转重新读取神经网络提供的模板和坐标;
                        case 3:   //结束本次动态追踪;
                        {
                            track_flags=0;
                            read_dp_flags=0;
                            for(int k=0; k < 3; k++)
				            {
					            snd_uart(me_fd, 0x04, 1, 0, 0, 0, 0);//拍照完成标记。
					            usleep(100000);
				            }
                        }
                            continue;                      
                        default:
                            continue;
                    }
                }
                continue;
            case 6:  //接受到结束吊舱的指令;
            {    
                snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500); //发送相机停止俯仰航向指令;
                thr_track_destroy(pthr_id,fd_uart);
                pthread_exit((void *)1);//结束吊舱程序1;
            }
                break;
            case 7:  //接受到关闭TX1电源的指令;
            {
                snd_uart(fd_uart, 0x32, 1500, 0, 0, 0x31, 1500); //发送相机停止俯仰航向指令;
                thr_track_destroy(pthr_id,fd_uart);
                pthread_exit((void *)2);//关机2;    
            }
                break;
            case 3:     //从串口获取变焦变化信息
                cam_size_update(uart_rcv_buf, camsize_area);
            
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
