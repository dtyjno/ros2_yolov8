这是一个简单的ros2-yolov8的代码
实现了从SIYI相机中拉流，并对拉流结果进行实时处理，同时发布原图像
以及发布识别目标的坐标。

 你可以使用

`conda install environment.yaml `
来安装环境

同时如果显示ultralytics库缺失

你可以使用

`pip install ultralytics`

在编译的时候，如果出现编译不过的情况
你可以先编译自定义消息类型功能包再编译。

如果你使用的是SIYI HM30 的卡录FPV摄像头以及图传
RTMP地址写在.sh里:rtsp://192.168.144.25:8554/main.264
在使用脚本时注意conda环境名称，路径等问题。

