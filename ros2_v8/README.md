这是一个简单的ros2-yolov8的代码
实现了从SIYI相机中拉流，并对拉流结果进行实时处理，同时发布原图像
以及发布识别目标的坐标。当然，还加上了舵机的控制与激光雷达的读数控制。


 你可以使用

`conda install environment.yaml `
来安装环境

我的环境是3,10的，如果你用的是3.12

可以使用vision_ros2jazzy_environment.yml来安装


同时如果显示ultralytics库缺失

你可以使用

`pip install ultralytics`

在编译的时候，如果出现编译不过的情况
你可以先编译自定义消息类型功能包再编译。（老分支）



如果你使用的是SIYI HM30 的卡录FPV摄像头以及图传
RTMP地址写在.sh里
在使用脚本时注意conda环境名称，路径等问题。

代码中记得也要改
可以从setup.py看一下测试代码的调用
包含了飞行状态的发布，舵机测试，实际帧数统计，坐标监听等

