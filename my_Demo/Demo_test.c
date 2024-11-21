#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>

#include "osal.h"
#include "oshw.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"


#define MOTORS_COUNT 2 // max 6 motors



char IOmap[4096];

typedef struct {
    int32 target_position;
    uint32 digital_outputs;
    uint16 control_word;
} output_t;

typedef struct {
    int32 actual_position;
    uint32 digital_inputs;
    uint16 status_word;
} input_t;

output_t *outputs[MOTORS_COUNT];
input_t *inputs[MOTORS_COUNT];

int run = 1;

void signal_handler(int sig) {
    (void)sig;
    run = 0;
    printf("Stopping EtherCAT...\n");
}
//状态切换
void slave_setup(uint16 slave) {
    int timeout = 0;
    printf("正在切换从站 %d 到 OPERATIONAL 状态...\n", slave);
    
    // 首先切换到INIT状态
    ec_slave[slave].state = EC_STATE_INIT;
    ec_writestate(slave);
    ec_statecheck(slave, EC_STATE_INIT, EC_TIMEOUTSTATE);
    
    // 切换到PRE-OP状态
    ec_slave[slave].state = EC_STATE_PRE_OP;
    ec_writestate(slave);
    ec_statecheck(slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    
    // 切换到SAFE-OP状态
    ec_slave[slave].state = EC_STATE_SAFE_OP;
    ec_writestate(slave);
    ec_statecheck(slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    
    // 最后切换到OP状态
    ec_slave[slave].state = EC_STATE_OPERATIONAL;
    ec_writestate(slave);
    
    // 等待并检查是否达到OP状态
    while(timeout < 10 && ec_slave[slave].state != EC_STATE_OPERATIONAL) {
        ec_statecheck(slave, EC_STATE_OPERATIONAL, 50000);
        printf("当前状态: 0x%x\n", ec_slave[slave].state);
        timeout++;
    }
}

//协议映射
int pdo_config(uint16 slave) {
    uint8 value8 = 0;
    uint32 value32;

    // Set Profile Position Mode
    uint8_t mode = 0x01;
    ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(uint8_t), &mode, EC_TIMEOUTRXM);

    //轮廓速度
    uint32_t profile_velocity = 30000;
    ec_SDOwrite(slave, 0x6081, 0x00, FALSE, sizeof(uint32_t), &profile_velocity, EC_TIMEOUTRXM);

    //轮廓加速度
    uint32_t profile_acceleration = 15566;
    ec_SDOwrite(slave, 0x6083, 0x00, FALSE, sizeof(uint32_t), &profile_acceleration, EC_TIMEOUTRXM);

    //轮廓减速度
    uint32_t profile_deceleration = 15566;
    ec_SDOwrite(slave, 0x6084, 0x00, FALSE, sizeof(uint32_t), &profile_deceleration, EC_TIMEOUTRXM);

    // Configure RPDO (0x1600)
    ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(uint8_t), &value8, EC_TIMEOUTRXM);
    
    value32 = 0x607A0020;  // Target Position
    ec_SDOwrite(slave, 0x1600, 1, FALSE, sizeof(uint32_t), &value32, EC_TIMEOUTRXM);
    
    value32 = 0x60FE0020;  // Digital outputs
    ec_SDOwrite(slave, 0x1600, 2, FALSE, sizeof(uint32_t), &value32, EC_TIMEOUTRXM);
    
    value32 = 0x60400010;  // Control Word
    ec_SDOwrite(slave, 0x1600, 3, FALSE, sizeof(uint32_t), &value32, EC_TIMEOUTRXM);
    
    value8 = 3;  // Number of entries
    ec_SDOwrite(slave, 0x1600, 0, FALSE, sizeof(uint8_t), &value8, EC_TIMEOUTRXM);

    // Configure TPDO (0x1A00)
    value8 = 0;
    ec_SDOwrite(slave, 0x1A00, 0, FALSE, sizeof(uint8_t), &value8, EC_TIMEOUTRXM);
    
    value32 = 0x60640020;  // Position Actual Value
    ec_SDOwrite(slave, 0x1A00, 1, FALSE, sizeof(uint32_t), &value32, EC_TIMEOUTRXM);
    
    value32 = 0x60FD0020;  // Digital inputs
    ec_SDOwrite(slave, 0x1A00, 2, FALSE, sizeof(uint32_t), &value32, EC_TIMEOUTRXM);
    
    value32 = 0x60410010;  // Status Word
    ec_SDOwrite(slave, 0x1A00, 3, FALSE, sizeof(uint32_t), &value32, EC_TIMEOUTRXM);
    
    value8 = 3;  // Number of entries
    ec_SDOwrite(slave, 0x1A00, 0, FALSE, sizeof(uint8_t), &value8, EC_TIMEOUTRXM);

    

    return 1;
}
//电机控制逻辑
void motor_control() {
    if (ec_init("enp0s25")) {
        if (ec_config_init(FALSE) > 0) {
            printf("Found %d slaves\n", ec_slavecount);
            
            ec_slave[1].PO2SOconfig = pdo_config;
            ec_slave[2].PO2SOconfig = pdo_config;
            ec_config_map(&IOmap);
            
            ec_configdc();
            ec_dcsync0(1, TRUE, 500000, 0);
            ec_dcsync0(2, TRUE, 500000, 0);
            
            slave_setup(0);
            // slave_setup(1);
            // slave_setup(2);
            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                // 先打印调试信息
               

                // 然后再进行映射
                outputs[0] = (output_t *)ec_slave[1].outputs;
                inputs[0] = (input_t *)ec_slave[1].inputs;

                outputs[1] = (output_t *)ec_slave[2].outputs;
                inputs[1] = (input_t *)ec_slave[2].inputs;
            

                
                while(run) {
                    ec_receive_processdata(EC_TIMEOUTRET);
                    
                    for(uint8_t i=0; i<MOTORS_COUNT; i++) {
                        uint16 status = inputs[i]->status_word;
                        
                        printf("\nmoto:%d,Status: 0x%04x, Control: 0x%04x, Target: %d, Current: %d", 
                           i,
                           status,
                           outputs[i]->control_word,
                           outputs[i]->target_position,
                           inputs[i]->actual_position);
                            
                        switch(status) {
                            case 0x1208:  // 故障状态
                                outputs[i]->control_word = 0x80;  // 先发送故障复位
                                ec_send_processdata();
                                break;
                                
                            case 0x1250:  // Ready to switch on
                                outputs[i]->control_word = 0x06;
                                outputs[i]->target_position = inputs[i]->actual_position;
                                ec_send_processdata();
                                break;
                                
                            case 0x1231:  // Switched on
                                outputs[i]->control_word = 0x07;
                                ec_send_processdata();
                                break;
                                
                            case 0x1233:  // Operation enabled
                                outputs[i]->control_word = 0x0F;
                                ec_send_processdata();
                                break;
                                
                            case 0x1237:  // Operation enabled + target reached
                            case 0x1637:
                                outputs[i]->target_position =167000;
                                outputs[i]->control_word = 0x1F;
                                ec_send_processdata();
                                break;
                                
                            default:
                                outputs[i]->control_word = 0x06;
                                ec_send_processdata();
                                break;
                            //延时10us
                            usleep(10);
                        }
                    }

                }
            }
        }
    }
}


                    


int main() {
    printf("SOEM Motor Control\n");
    
    signal(SIGINT, signal_handler);
    
    
        motor_control();
     
    
    return 0;
}
