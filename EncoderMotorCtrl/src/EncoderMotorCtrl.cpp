#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "EncoderMotorCtrl.h"

/*V1(一層エンコーダ)*/

// その名の通りセットアップを行います。それだけです
// 引数の役割がなんなのか知りたかったらサンプルスケッチを見てください
void EncoderMotorCtrl::setup(int dir, int pwm, int gpio, pcnt_unit_t pcnt_unit)
{

    pinMode(dir, OUTPUT);
    pinMode(pwm, OUTPUT);

    digitalWrite(dir, LOW);
    digitalWrite(pwm, LOW);

    direction_pin = dir;
    pwm_pin = pwm;
    UNIT = pcnt_unit;

    pcnt_config_t pcnt_conf = {

        .pulse_gpio_num = gpio,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = -32767,
        .unit = pcnt_unit,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_unit_config(&pcnt_conf);

    pcnt_set_filter_value(pcnt_unit, 1023); // 指定したクロック以下のパルスを無視
    pcnt_filter_enable(pcnt_unit);

    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    vTaskDelay(pdMS_TO_TICKS(100));
    pcnt_counter_resume(pcnt_unit);
}

// 一相エンコーダーの為モーターの回転方向が取得できず、エンコーダーの値の増減ができません。
// そこでこの関数を用いて逆転の場合は値を減少させる処理を入れています。
// set_locate()関数を実行した場合に自動的に呼び出されるのでこちらを実行する必要はありません。
void EncoderMotorCtrl::set_direction(pcnt_unit_t unit, bool direction)
{
    pcnt_counter_pause(unit);
    if (direction)
    {
        // 正転時はカウントアップ
        pcnt_set_mode(unit, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_DIS, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    }
    else
    {
        // 逆転時はカウントダウン（INCをDECに変更）
        pcnt_set_mode(unit, PCNT_CHANNEL_0, PCNT_COUNT_DEC, PCNT_COUNT_DIS, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    }
    pcnt_counter_resume(unit);
}

// 引数で指定した位置までモーターを回転させます。ただそれだけです
void EncoderMotorCtrl::set_locate(int goal_locate)
{

    stop_set_speed();

    while (1)
    {

        pcnt_get_counter_value(UNIT, &counter_value);
        int dif = goal_locate - counter_value;

        if (int(abs(dif)) < 1)
            break;

        bool direction = (dif > 0);

        // 方向が変わった時だけモードを切り替える
        if (direction != last_direction)
        {
            set_direction(UNIT, direction);
            last_direction = direction;
            delay(1); // 安定待ち
        }

        digitalWrite(direction_pin, direction ? HIGH : LOW);

        analogWrite(pwm_pin, constrain(abs(int(dif * p_gain)), 5, 255));

        // デバッグ
        if (debug_mode)
        {
            Serial.printf("目標位置 : %d, 現在位置 : %d, 現在速度 : %d\n", goal_locate, counter_value, constrain(abs(int(dif * p_gain)), 10, 255));
        }
        delay(10);
    }
    analogWrite(pwm_pin, 0);
    return;
}

/*V2(二層エンコーダ)*/

void EncoderMotorCtrl::setup_v2(int dir, int pwm, int gpioA, int gpioB, pcnt_unit_t pcnt_unit)
{

    pinMode(dir, OUTPUT);
    pinMode(pwm, OUTPUT);

    direction_pin = dir;
    pwm_pin = pwm;
    UNIT = pcnt_unit;

    pcnt_config_t pcnt_conf = {

        .pulse_gpio_num = gpioA,
        .ctrl_gpio_num = gpioB,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = -32767,
        .unit = pcnt_unit,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_unit_config(&pcnt_conf);

    pcnt_set_filter_value(pcnt_unit, 1023); // 指定したクロック以下のパルスを無視
    pcnt_filter_enable(pcnt_unit);

    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    vTaskDelay(pdMS_TO_TICKS(100));
    pcnt_counter_resume(pcnt_unit);
}

void EncoderMotorCtrl::set_locate_v2(int goal_locate)
{

    while (1)
    {

        pcnt_get_counter_value(UNIT, &counter_value);
        int dif = goal_locate - counter_value;

        if (int(abs(dif)) < 1)
            break;

        bool direction = (dif > 0);

        digitalWrite(direction_pin, direction ? HIGH : LOW);

        analogWrite(pwm_pin, constrain(abs(int(dif * p_gain)), 5, 255));

        // デバッグ
        if (debug_mode)
        {
            Serial.printf("目標位置 : %d, 現在位置 : %d, 現在速度 : %d\n", goal_locate, counter_value, constrain(abs(int(dif * p_gain)), 10, 255));
        }
        delay(10);
    }
    analogWrite(pwm_pin, 0);
    return;
}

// タスク作成が上手くいかないので引数のスピードをアナログ出力するだけの非常に簡素な関数となっています。
void EncoderMotorCtrl::set_speed_v2(int speed)
{

    digitalWrite(direction_pin, speed > 0 ? HIGH : LOW);
    analogWrite(pwm_pin, constrain(abs(speed), 0, 255));
}

/*両方*/

// 現在のエンコーダーの値を読み取る関数です
// 正直なところset_locate関数を実行している間は常に値の更新がされているのでわざわざこれを呼び出す必要はないです
// 移動が完了した後に確認するという用途ならいいかもしれません。
int EncoderMotorCtrl::get_value()
{
    pcnt_get_counter_value(UNIT, &counter_value);
    return counter_value;
}

// デバッグモードのオンオフを切り替える関数です。デバッグモードがオンの時はset_locate関数を実行している間にシリアルモニターに現在の位置と目標位置、現在の速度が表示されます。
void EncoderMotorCtrl::debug(bool mode)
{
    debug_mode = mode;
}

// エンコーダーの値をリセットする関数です。ただそれだけです。
void EncoderMotorCtrl::locate_reset()
{
    pcnt_counter_pause(UNIT);
    pcnt_counter_clear(UNIT);
    vTaskDelay(pdMS_TO_TICKS(100));
    pcnt_counter_resume(UNIT);
}

// P制御のゲイン(倍率)を決めるための関数。単に実行しただけだとデフォルト値が代入されます。
void EncoderMotorCtrl::gain_setup(float kp)
{
    p_gain = kp;
}

// タスク作成が上手くいかないので引数のスピードをアナログ出力するだけの非常に簡素な関数となっています。
void EncoderMotorCtrl::set_speed(int speed)
{

    // digitalWrite(direction_pin, speed > 0 ? HIGH : LOW);
    // analogWrite(pwm_pin, constrain(abs(speed), 0, 255));

    if (!speed_ctrl_running)
    {
        target_speed = speed;
        xTaskCreate(speed_ctrl_task, "speed_ctrl_task", 2048, this, 1, &speed_ctrl_handle);
        speed_ctrl_running = true;
    }
}

void EncoderMotorCtrl::speed_ctrl_task(void *pvParameters)
{
    EncoderMotorCtrl *motor = (EncoderMotorCtrl *)pvParameters;

    int start_rpm = 0;
    int last_rpm = 0;
    int dif = 0;
    int rpm = 0;
    int rpm_dif = 0;

    while (1)
    {

        start_rpm = motor->get_value();
        vTaskDelay(pdMS_TO_TICKS(100));
        last_rpm = motor->get_value();
        dif = abs(last_rpm - start_rpm);
        rpm = dif * 600 / 1024; // 600は100msあたりの回転数に変換するための係数、1024はエンコーダーの分解能

        rpm_dif = motor->target_speed - rpm;

        digitalWrite(motor->direction_pin, motor->target_speed > 0 ? HIGH : LOW);
        analogWrite(motor->pwm_pin, constrain(abs(motor->target_speed + (rpm_dif * motor->p_gain)), 0, 255));

        // デバッグ
        if (motor->debug_mode)
        {
            Serial.printf("目標速度 : %d, 現在速度 : %d\n", motor->target_speed, rpm);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void EncoderMotorCtrl::stop_set_speed()
{

    if (speed_ctrl_running)
    {
        vTaskDelete(speed_ctrl_handle);
        speed_ctrl_running = false;

        digitalWrite(direction_pin, LOW);
        analogWrite(pwm_pin, 0);

        // デバッグ
        if (debug_mode)
        {
            Serial.printf("速度制御を停止しました。\n");
        }
    }
}
