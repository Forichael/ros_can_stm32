## ros\_can_stm32 ##

这个是ROS机器人的底盘源代码文件，采用keil进行开发，使用了robomoudle驱动器以及stm32f103rc单片机。

非常感谢**Book诗意**作者的底盘文件开源，从他那我得到了许多灵感，在按照他人的思路进行开发的时候，也发现作者源代码存在的一些问题，也与其本人进行探讨过改进思路。

我在作者的思路基础上，对帧数据增加了CRC校验，使得在ROS上位机读取编码器数据的时候能够精确读取，减少了帧数据错误的发生。
在作者使用robomoudle驱动器的思路上，使用驱动器自带的CAN通讯口进行通讯，使得数据的收发更精确。

再次感谢**Book诗意**，也非常感谢**尚程功夫**提供的STM32串口通讯的demo。

如有不足，欢迎大家指正。

传送门：

[Book诗意](http://bookshiyi.com/ "Book诗意")


微信公众号：尚程功夫

我的联系方式：

Email：forichael.ouyang@outlook.com


QQ：971405784