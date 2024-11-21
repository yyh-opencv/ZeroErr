主程序在my_Demo里面
依次运行
mkdir build
cd build && cmake ..
make 

注意点：1.需要更改电机数目：static const uint8_t MOTOR_COUNT = 2; //电机数目，需要看情况改，必须匹配，或库函数查询设备数目也行
       2.需要修改网口名称：ifconig能看： 
       // 创建控制器实例
    controller = new EthercatControler("enp0s25");
    if(!controller->EthInit()) {
        std::cout << "init failed" << std::endl;
        delete controller;
        return -1;
    }

  TMD，什么垃圾电机，就b站up主糖晴豆有开源，但也有bug，看了很多遍协议才找出问题。还容易发烫，小电池供电不稳，丧失记忆，关键还买的死贵，不如喵喵电机和大然。

嘿嘿！我也有bug，运行发送和设置目标，我设为两个线程，但初始化获取当前位置设为位置，会造成另外线程的设置目标失败。有大佬欢迎来踢。
