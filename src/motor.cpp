#include "motor.hpp"

int pix_move(int height, int direction, int camsize, int *movcam)
{
    //speed pwm,times 时间单位ms
    int speed,times;

    switch(camsize)
    {
        case 1:
        case 2:
        {
            if(direction==1)//向下移动;
            {
                speed=1160;
                times=((11000*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1850;
                times=((11000*height)/720);
            }
            break;
        }
        case 3:
        {
            if(direction==1)//向下移动;
            {
                speed=1150;
                times=((10500*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1850;
                times=((10700*height)/720);
            }
            break;
        }
        case 4:
        {
            if(direction==1)//向下移动;
            {
                speed=1150;
                times=((9000*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1850;
                times=((9200*height)/720);
            }
            break;
        }
        case 5:
        {
            if(direction==1)//向下移动;
            {
                speed=1150;
                times=((7800*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1850;
                times=((8000*height)/720);
            }
            break;
        }
        case 6:
        {
            if(direction==1)//向下移动;
            {
                speed=1150;
                times=((6600*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1850;
                times=((6700*height)/720);
            }

            break;
        }
        case 7:
        {
            if(direction==1)//向下移动;
            {
                speed=1250;
                times=((8200*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1750;
                times=((8600*height)/720);
            }
            break;
        }
        case 8:
        {
            if(direction==1)//向下移动;
            {
                speed=1250;
                times=((6800*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1750;
                times=((7000*height)/720);
            }
            break;
        }
        case 9:
        {
            if(direction==1)//向下移动;
            {
                speed=1250;
                times=((6000*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1750;
                times=((6000*height)/720);
            }
            break;
        }
        case 10:
        {
            if(direction==1)//向下移动;
            {
                speed=1300;
                times=((7000*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1700;
                times=((7200*height)/720);
            }
            break;
        }
        case 11:
        {
            if(direction==1)//向下移动;
            {
                speed=1300;
                times=((6000*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1700;
                times=((6200*height)/720);
            }
            break;
        }
        case 12:
        {
            if(direction==1)//向下移动;
            {
                speed=1300;
                times=((4900*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1700;
                times=((5100*height)/720);
            }
            break;
        }
        case 13:
        {
            if(direction==1)//向下移动;
            {
                speed=1350;
                times=((6500*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1650;
                times=((6700*height)/720);
            }
            break;
        }
        case 14:
        {
            if(direction==1)//向下移动;
            {
                speed=1350;
                times=((5800*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1650;
                times=((6000*height)/720);
            }
            break;
        }
        case 15:
        {
            if(direction==1)//向下移动;
            {
                speed=1350;
                times=((4900*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1650;
                times=((5100*height)/720);
            }
            break;
        }
        case 16:
        {
            if(direction==1)//向下移动;
            {
                speed=1400;
                times=((7200*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1600;
                times=((8000*height)/720);
            }
            break;
        }
        case 17:
        {
            if(direction==1)//向下移动;
            {
                speed=1400;
                times=((6500*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1600;
                times=((7600*height)/720);
            }
            break;
        }
        case 18:
        {
            if(direction==1)//向下移动;
            {
                speed=1400;
                times=((5600*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1600;
                times=((6200*height)/720);
            }
            break;
        }
        case 19:
        {
            if(direction==1)//向下移动;
            {
                speed=1420;
                times=((7300*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1590;
                times=((7100*height)/720);
            }
            break;
        }
        case 20:
        {
            if(direction==1)//向下移动;
            {
                speed=1420;
                times=((6400*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1580;
                times=((7600*height)/720);
            }
            break;
        }
        case 21:
        {
            if(direction==1)//向下移动;
            {
                speed=1420;
                times=((5700*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1580;
                times=((6800*height)/720);
            }
            break;
        }
        case 22:
        {
            if(direction==1)//向下移动;
            {
                speed=1420;
                times=((4900*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1580;
                times=((5900*height)/720);
            }
            break;
        }
        case 23:
        {
            if(direction==1)//向下移动;
            {
                speed=1430;
                times=((6000*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1580;
                times=((5100*height)/720);
            }
            break;
        }
        case 24:
        {
            if(direction==1)//向下移动;
            {
                speed=1435;
                times=((7400*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1575;
                times=((6200*height)/720);
            }
            break;
        }
        case 25:
        {
            if(direction==1)//向下移动;
            {
                speed=1435;
                times=((6500*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1570;
                times=((6200*height)/720);
            }
            break;
        }
        case 26:
        {
            if(direction==1)//向下移动;
            {
                speed=1435;
                times=((5600*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1570;
                times=((5200*height)/720);
            }
            break;
        }
        case 27:
        {
            if(direction==1)//向下移动;
            {
                speed=1440;
                times=((5800*height)/720);
            }
            else if(direction==2)//向上移动;
            {
                speed=1565;
                times=((7500*height)/720);
            }
            break;
        }
        case 28:
        case 29:
        case 30:
        {
            break;
        }
        
    }

    *movcam=speed;
    *(movcam+1)=times;
    
    return 0;
}
