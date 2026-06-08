/*エンコーダー付きのDCモーターをESP32で制御することを前提に作られたライブラリです。ESP32以外では使用できないのでご注意ください。*/
#ifndef EncoderMotorCtrl_H_
#define EncoderMotorCtrl_H_

#include <Arduino.h>
#include "driver/pcnt.h"

class EncoderMotorCtrl
{

public:

    /*V1*/
    void setup(int dir, int pwm, int gpio, pcnt_unit_t pcnt_unit = PCNT_UNIT_0);

    void set_locate(int goal_locate);
  
    void set_direction(pcnt_unit_t unit, bool direction);


    /*V2*/

    void setup_v2(int dir, int pwm, int gpioA, int gpioB, pcnt_unit_t pcnt_unit = PCNT_UNIT_0);

    void set_locate_v2(int goal_locate);

    void set_speed_v2(int speed);

    /*両方*/

    int get_value();

    void set_speed(int speed);

    void stop_set_speed();

    void debug(bool mode);

    void locate_reset();

    void gain_setup(float kp = 5.0f);

private:
    int direction_pin = 0, pwm_pin = 0;

    int16_t counter_value = 0, last_counter_value = 0;
    float p_gain = 5.0f, i_gain = 0.0f, d_gain = 0.0f;
    bool last_direction = true;
    pcnt_unit_t UNIT = {};

    bool debug_mode = false;

    /*speed_ctrl_task*/
    TaskHandle_t speed_ctrl_handle = NULL;
    int target_speed = 0;
    bool speed_ctrl_running = false;

    static void speed_ctrl_task(void *pvParameters);

};

#endif
