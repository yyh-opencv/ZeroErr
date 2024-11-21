#include <Ethercat_control.hpp>
#include <iostream>
#include <chrono>
#include <thread>


EthercatControler::EthercatControler(const std::string& ifname) : ifname_(ifname),running_(false)
{
    for(int i =0;i<=MOTOR_COUNT;i++){
        outputs_[i] = nullptr;    
        inputs_[i] = nullptr;
    }
}


EthercatControler::~EthercatControler()
{
    //0代表所有设备
    stop();
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
}



/******
 if (ec_init("enp0s25")) {
        if (ec_config_init(FALSE) > 0) {
            printf("Found %d slaves\n", ec_slavecount);
            
            ec_slave[1].PO2SOconfig = pdo_config;
            ec_slave[2].PO2SOconfig = pdo_config;
            ec_config_map(&IOmap);
            
            ec_configdc();
            ec_dcsync0(1, TRUE, 500000, 0);
            ec_dcsync0(2, TRUE, 500000, 0);
*******/
bool EthercatControler::EthInit()
{
    if(!ec_init(ifname_.c_str())){
        std::cout<<"False to init ethernet master"<<std::endl;
        return false;
    }
    
   if(ec_config_init(FALSE) <= 0) {
    
        return false;
    }


    for(int i =1;i<=MOTOR_COUNT;i++){
        ec_slave[i].PO2SOconfig = pdoConfig;
        std::cout<<"pdoConfig: "<< i <<std::endl;
    }

    ec_config_map(&IO_map);
    ec_configdc();

    for(int i =1;i<=MOTOR_COUNT;i++){
        ec_dcsync0(i,TRUE,500000,0);
    }

    slaveSetup(0); //所有设备切换状态

    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        for(int i = 1; i <= MOTOR_COUNT; i++) {
            outputs_[i] = (MotorOutput*)ec_slave[i].outputs;
            inputs_[i] = (MotorInput*)ec_slave[i].inputs;

            std::cout << "电机" << i << " PDO映射地址:"
              << " outputs=" << outputs_[i]
              << " inputs=" << inputs_[i] << std::endl;


            if(outputs_[i]==nullptr||inputs_[i]==nullptr){
               std::cout<<"电机"<<i <<"PDO映射失败"<<std::endl; 
            }
            outputs_[i]->control_word = 0x0000;
            outputs_[i]->target_position = 0;
        }
        return true;
    }
    return false;

}

void EthercatControler::slaveSetup(uint8_t slave)
{
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


int EthercatControler::pdoConfig(uint16_t slave)
{
    uint8 value8 = 0;
    uint32 value32;

    // Set Profile Position Mode
    uint8_t mode = 0x01; //轮廓速度模式
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

void EthercatControler::processData(){
    ec_receive_processdata(EC_TIMEOUTRET);

    static int count = 0;
    for(int i = 1;i<=MOTOR_COUNT;i++){
        if(inputs_[i]==nullptr){
            std::cout<<"电机"<<i<<"inputs_为空"<<std::endl;
            continue;
        }
        uint16_t status = inputs_[i]->status_word;
        
        
   
        switch(status) {
            case 0x1208:  // 故障状态
                outputs_[i]->control_word = 0x80;  // 故障复位
                
                break;
                
            case 0x1250:  // 准备使能
                outputs_[i]->control_word = 0x06;
                
                break;
                
            case 0x1231:  // 已切换使能
                outputs_[i]->control_word = 0x07;
                
                break;
                
            case 0x1233: // 操作使能
                
                outputs_[i]->control_word = 0x0F;
                
                
                // outputs_[i]->target_position = inputs_[i]->actual_position;
                   
                // std::cout << "电机" << i << "初始位置: " << inputs_[i]->actual_position << std::endl;
                
            break;


                
            case 0x1237:  // 运行状态
            case 0x1637:
                outputs_[i]->control_word = 0x1F;
                
                break;
                
            default:
                outputs_[i]->control_word = 0x06;
                
                break;
        }
        
        
            std::cout << "电机 " << i << " 状态: 0x" << std::hex << status 
                     << " 位置: " << std::dec << inputs_[i]->actual_position
                     <<"目标位置："<<std::dec<<outputs_[i]->target_position << std::endl;
        
       
    }
    
    ec_send_processdata();
    std::this_thread::sleep_for(std::chrono::microseconds(500));//ethercat 是高速通信，延时1ms电机都不能正常通信
}


void EthercatControler::set_Motors_TargetPosition(uint8_t motor_id, int32_t target_pos)
{
    if(motor_id >= 1 && motor_id <= MOTOR_COUNT) {
        std::cout << "设置电机 " << (int)motor_id << " 目标位置: " << target_pos << std::endl;
        outputs_[motor_id]->target_position = target_pos;
        
    } else {
        std::cout << "无效的电机ID: " << (int)motor_id << std::endl;
    }
}




void EthercatControler::stop()
{
    std::cout << "停止运动...."<<std::endl;
    running_ = false;
}

void EthercatControler::run()
{
    std::cout << "开始运动...."<<std::endl;
    running_ = true;
    while(running_) {
        processData();
    }

}
uint16_t EthercatControler :: get_motor_status(uint8_t motor_id) {
    if(motor_id >= 1 && motor_id <= MOTOR_COUNT) {
        return inputs_[motor_id]->status_word;
    }
    return 0;
}



