#ifndef ETHERCAT_CONTROL_H
#define ETHERCAT_CONTROL_H
#include <cstdint>
#include <string>
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

class EthercatControler
{
public:
    EthercatControler(const std::string& ifname);
    ~EthercatControler();

    bool EthInit();
    void set_Motors_TargetPosition(uint8_t motor_id, int32_t target_pos);
    uint16_t get_motor_status(uint8_t motor_id);
    void run();
    void stop();

      static const uint8_t MOTOR_COUNT = 2; //电机数目，需要看情况改，必须匹配，或库函数查询设备数目也行
private:
    void slaveSetup(uint8_t slave);
    static int pdoConfig(uint16_t slave);
    void processData();
    

    struct MotorOutput{
        int32_t target_position;
        uint32_t digital_output;
        uint16_t control_word;
    };

    struct MotorInput{
        int32 actual_position;
        uint32 digital_inputs;
        uint16 status_word;
    };

  
    std::string ifname_;
    char IO_map[4096];
    bool running_;
    MotorOutput* outputs_[MOTOR_COUNT+1];
    MotorInput* inputs_[MOTOR_COUNT+1];
};



#endif
