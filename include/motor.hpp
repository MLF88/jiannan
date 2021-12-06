#ifndef MOTOR_H__
#define MOTOR_H__

//height 目标高度，direction 相机运动方向，1 代表向下，2代表向上。
//camsize 当前焦距;
int pix_move(int height, int direction, int camsize, int *movcam);
#endif