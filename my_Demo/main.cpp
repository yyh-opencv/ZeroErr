#include <Ethercat_control.hpp>
#include <iostream>
#include <signal.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sched.h>
#include <string.h>
#include <chrono>
#include <thread>

#define MY_STACK_SIZE 8192
EthercatControler* controller = nullptr;


void signal_handler(int signum)
{
    if(controller){
        std::cout << "接收到中断信号,Stop the controller" << std::endl;
        controller->stop();
    }
}

void* rt_task(void *arg)
{
    std::cout << "RT Task Start" << std::endl;
    controller->run();
    return nullptr;
}


/***********if (ec_init("enp0s25"))***** */
int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);

    // 锁定内存
    if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cout << "mlockall failed" << std::endl;
        return -1;
    }

    // 创建控制器实例
    controller = new EthercatControler("enp0s25");
    if(!controller->EthInit()) {
        std::cout << "init failed" << std::endl;
        delete controller;
        return -1;
    }

    // 创建实时线程
    pthread_t thread;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN+MY_STACK_SIZE);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 80;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    
    

    // 启动实时任务
    int ret = pthread_create(&thread, &attr, rt_task, nullptr);
    if(ret) {
        std::cout << "pthread_create failed" << strerror(ret) << std::endl;
        delete controller;
        return -1;
    }
    controller->set_Motors_TargetPosition(1, 0);
    controller->set_Motors_TargetPosition(2, 0);
   

    

    pthread_setname_np(thread, "ethercat_task");
    pthread_join(thread, nullptr);

    delete controller;
    return 0;
}

    

