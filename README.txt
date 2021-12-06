程序支持绝缘子及下挂点的拍摄，模型包含绝缘子及绝缘子下挂点，

1.依赖环境
1）. 主机支持HDMI相机视频采集。
2）. caffe-ssd配置环境


2.caffe-ssd模型问题
文件dp_argu.hpp 定义使用caffe model，输入caffemodel和deploy.protxt的位置。
其中训练模型过程中
labelmap_voc.prototxt 文件

挂点
name: "hangingpoint"
label: 2
绝缘子
name: "insulator"
label: 3

另：
默认下 model文件夹存放 *.caffemodel,deploy.protxt文件
data文件夹存放 模型输出结果保存成图片

3.程序编译
cd ~/jinan_insu
make clean
make
生成 all 可执行文件
./all  即可运行程序
