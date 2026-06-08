#include <EncoderMotorCtrl.h>
#include <driver/pcnt.h>

EncoderMotorCtrl motor;

void setup() {

  Serial.begin(115200);

  //(方向出力ピン, PWM出力ピン, カウントピンA, カウントピンB, 使用ユニット(デフォルトPCNT_UNIT_0), Pゲイン(デフォルト5.0f))
  motor.setup_v2(17, 18, 19, 21);
}

void loop() {

  //目標位置を設定
  motor.set_speed(100);

  //現在のエンコーダーの値を出力
  Serial.printf("motor:%d\n", motor.get_value());

  delay(5000);

  motor.set_speed(255);

  delay(5000);
}
