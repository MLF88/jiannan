#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>

#include <fstream>
#include <iostream>


#include "camera.hpp"


int h;
int framesize;
int fd;

struct buffer *buffers;

static int n_buffers = 0;

/******function**********/
static void errno_exit(const char *s)
{
	fprintf(stderr, "error %d, %d, %s\n", errno, (*s), strerror(errno));
	exit(EXIT_FAILURE);
}
static int xioctl(int fd, int request, void *arg)
{
	int r;
	do
	{
		r = ioctl(fd, request, arg);
	}
	while (-1 == r && EINTR == errno);
	return r;
}
static void open_device(char *dev_name)
{

	h = IMAGEHEIGHT*3/2;
	framesize = IMAGEHEIGHT*IMAGEWIDTH*3/2;
	fd = -1;
	buffers = NULL;	
	n_buffers = 0;
	
	struct stat st;
	if(-1 == stat(dev_name, &st))
	{
		fprintf(stderr,"Cannot identify %s:%d,%s\n",dev_name,errno,strerror(errno));
		exit(EXIT_FAILURE);
	}
	if(!S_ISCHR(st.st_mode))
	{
		fprintf(stderr,"%s is no device\n",dev_name);
		exit(EXIT_FAILURE);
	}

	fd = open(dev_name, O_RDWR, 0);
	if(-1 == fd)
	{
		fprintf(stderr, "Cannot open %s:%d,%s\n",dev_name,errno,strerror(errno));
		exit(EXIT_FAILURE);
	}
}
static void init_device_cap(void)
{
	struct v4l2_capability cap;

	if (-1 == xioctl(fd,VIDIOC_QUERYCAP, &cap))
	{
		if(EINVAL == errno)
		{
			fprintf(stderr," is no V4L2 devide\n");
			exit(EXIT_FAILURE);
		}
		else
		{
			printf("vidioc_querycap\n");
			errno_exit("VIDIOC_QUERYCAP");
		}
	}
	else
	{
		printf("driver:\t\t%s\n",cap.driver);
		printf("card:\t\t%s\n",cap.card);
		printf("bus_info:\t%s\n",cap.bus_info);
		printf("version:\t%d\n",cap.version);
		printf("capabilities:\t%x\n",cap.capabilities);	
	}

	if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
	{
		fprintf(stderr,"is no video capture devcie\n");
		exit(EXIT_FAILURE);
	}
	if(!(cap.capabilities & V4L2_CAP_STREAMING))
	{
		fprintf(stderr,"Device : supports streaming.\n");
		exit(EXIT_FAILURE);
	}
}

static void init_device_fmt(void)
{
	struct v4l2_format fmt;

	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
	fmt.fmt.pix.height = IMAGEHEIGHT;
	fmt.fmt.pix.width = IMAGEWIDTH;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

	if(-1 == xioctl(fd,VIDIOC_S_FMT, &fmt))
	{
		fprintf(stderr,"Unable to set format\n");
		errno_exit("VIDIOC_S_FMT");
	}
}

static void init_mmap(void)
{
	struct v4l2_requestbuffers req;
	CLEAR(req);
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if(-1 == xioctl(fd,VIDIOC_REQBUFS, &req))
	{
		if(EINVAL == errno)
		{
			fprintf(stderr,"does not support memory mapping\n");
			exit(EXIT_FAILURE);
		}
		else
		{
			printf("vidioc_reqbufs\n");
			errno_exit("VIDIOC_REQBUFS");
		}
	}
	if(req.count < 2)
	{
		fprintf(stderr,"Insufficient buffer memory on\n");
		exit(EXIT_FAILURE);
	}
	buffers = (buffer*)calloc(req.count, sizeof(buffer));
	if(!buffers)
	{
		fprintf(stderr,"Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for(n_buffers = 0; n_buffers < req.count; ++n_buffers)
	{
		struct v4l2_buffer buf;

		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;	
		if(-1 == xioctl(fd,VIDIOC_QUERYBUF,&buf)) 
		{
			printf("vidioc_querybuf\n");
			errno_exit("VIDIOC_QUERYBUF");
		}
		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = mmap(NULL,buf.length,
										PROT_READ | PROT_WRITE,
										MAP_SHARED,fd,buf.m.offset);
		if(MAP_FAILED == buffers[n_buffers].start)
		{
			printf("map_prinf\n");
			errno_exit("mmap");
		}

		if(-1 == xioctl(fd,VIDIOC_QBUF, &buf))
		{
			printf("vidioc_qbuf\n");
			errno_exit("VIDIOC_QBUF");
		}
	}
}
static void start_capturing(void)
{
	enum v4l2_buf_type types;
	types= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(-1 == xioctl(fd,VIDIOC_STREAMON,&types))
	{
		printf("vidioc_streamon\n");
		errno_exit("VIDIOC_STREAMON");
	}
	//yuvImg.create(h,IMAGEWIDTH,CV_8UC1);

	printf("start capturing ok\n");
}

void init_device(char *camerapath)
{
	open_device(camerapath);
	init_device_cap();
	//init_device_fmtdesc();
	init_device_fmt();
	init_mmap();
	start_capturing();
}

Mat read_frame(Mat yuvImg, Mat bgrImg)
{
	struct v4l2_buffer buf;
	CLEAR(buf);
	buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory=V4L2_MEMORY_MMAP;
	if(-1 == xioctl(fd,VIDIOC_DQBUF, &buf))
	{
		printf("vidioc_dqbuf\n");
		errno_exit("VIDIOC_DQBUF");
	}
	/* process_image */
	memcpy(yuvImg.data,buffers[buf.index].start,framesize*sizeof(unsigned char));
	cvtColor(yuvImg,bgrImg,CV_YUV2BGR_I420);

	if(-1 == xioctl(fd,VIDIOC_QBUF,&buf))
	{
		printf("back into the buffer queue failed\n");
		errno_exit("VIDIOC_QBUF");
	}

	return bgrImg;	
}



	
