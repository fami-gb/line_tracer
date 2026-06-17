#include <iostream>
#include <algorithm>
#include <cmath>
#include <array>

#include "kernel_cfg.h"
#include "app.h"
#include <stdio.h>
#include <stdlib.h>

#include <libcpp/spike/IMU.h>
#include <libcpp/spike/Display.h>
#include <libcpp/spike/ForceSensor.h>
#include <libcpp/spike/ColorSensor.h>
#include <libcpp/spike/Button.h>
#include <libcpp/spike/Clock.h>
#include <libcpp/spike/Motor.h>
#include <serial/serial.h>
#include <serial/newlib.h>
#include <syssvc/serial.h>

#include <libcpp/spike/Port.h>

extern "C" {
  void __attribute__((weak)) _fini() {}
  void* __dso_handle __attribute__((weak)) = (void*)-1;
}

namespace {
  using namespace spikeapi;
  Display g_display;
  Motor g_right_motor(EPort::PORT_A);
  Motor g_left_motor(EPort::PORT_B);
  ForceSensor g_forceSensor(EPort::PORT_D);
  ColorSensor g_colorSensor(EPort::PORT_E);
  ColorSensor::HSV g_hsv;
  Button g_button;
  Clock g_clock;
  FILE *fp;
  int targetBrightness = 0;
}

uint64_t msNow() { return g_clock.now() / 1000; }

void SerialCalibration() {
    // Bluetooth接続が確立されるまで待機
    bool connected = false;
    while (!connected) {
        ER err = serial_opn_por(SIO_BLUETOOTH_PORTID);
        if(err == E_OK) connected = true;
        fp = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);
        if (fp != nullptr) connected = true;
    }
}

void BrightnessCalibration() {
  fprintf(fp, "輝度測定を開始します。\n");
  int BlackBrightness = -1;
  int WhiteBrightness = -1;
  bool blackCalibrated = false;
  bool whiteCalibrated = false;
  g_display.showChar('B');
  fprintf(fp, "黒ラインの上にセンサを配置してください。\n");
  fprintf(fp, "左ボタンで黒の輝度を測定します(右ボタンで黒輝度を確定します)。\n");
  while (1) {
    if (g_button.isLeftPressed() && !blackCalibrated) {
      blackCalibrated = true;
      fprintf(fp, "黒の輝度を測定中...\n");
    }

    if (blackCalibrated) {
      BlackBrightness = g_colorSensor.getReflection();
      g_display.showNumber(BlackBrightness);

      if (g_button.isRightPressed()) {
        fprintf(fp, "黒の輝度測定が終了しました。\n");
        fprintf(fp, "黒の輝度: %d\n", BlackBrightness);
        break;
      }
    }
  }

  g_display.showChar('W');
  fprintf(fp, "白ラインの上にセンサを配置してください。\n");
  fprintf(fp, "左ボタンで白の輝度を測定します(右ボタンで白輝度を確定します)。\n");
  while (1) {
    if (g_button.isLeftPressed() && !whiteCalibrated) {
      whiteCalibrated = true;
      fprintf(fp, "白の輝度を測定中...\n");
    }

    if (whiteCalibrated) {
      WhiteBrightness = g_colorSensor.getReflection();
      g_display.showNumber(WhiteBrightness);

      if (g_button.isRightPressed()) {
        fprintf(fp, "白の輝度測定が終了しました。\n");
        fprintf(fp, "白の輝度: %d\n", WhiteBrightness);
        blackCalibrated = false;
        whiteCalibrated = false;
        break;
      }
    }
  }

  targetBrightness = (BlackBrightness + WhiteBrightness) / 2;
  fprintf(fp, "輝度のキャリブレーションが完了しました。\n");
  fprintf(fp, "目標輝度: %d\n", targetBrightness);
}

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {
  g_display.showChar('S'); // Serial
  SerialCalibration();
  g_display.showChar('E'); // End

  g_display.showChar('C'); // Color
  BrightnessCalibration();
  g_display.showChar('E'); // End

  // フォースセンサのボタンが押されるまで待機
  fprintf(fp, "フォースセンサを押してください\n");
  while(!g_forceSensor.isPressed(0.5f)) {} 
  g_clock.reset();
  fprintf(fp, "フォースセンサが押されました。\n");
  sta_cyc(TRACER_TASK_CYC); 
  ext_tsk();
}

double calculatePid(int error) {
  static uint64_t previousTime = 0;
  static double previousError = 0.0;
  static double integral = 0.0;
  static double previousDerivative = 0.0;
  const int alpha = 0.7;
  const double Kp = 1.0; // 比例ゲイン
  const double Ki = 0.1; // 積分ゲイン
  const double Kd = 0.05; // 微分ゲイン

  uint64_t currentTime = msNow() / 1000.0; // 現在の時間を秒単位で取得
  uint64_t dt = dt ? currentTime - previousTime : 0.008; // 前回の時間からの経過時間を計算

  integral += error * dt; // 積分項の計算
  double derivative = (alpha * (error - previousError) / dt + (1 - alpha) * previousDerivative); // LPFを適用
  previousError = error; // 前回の誤差を更新
  previousDerivative = derivative; // 前回の微分を更新
  previousTime = currentTime; // 前回の時間を更新

  integral = std::max(std::min(integral, 100.0), -100.0); // 積分項の飽和制限
  derivative = std::max(std::min(derivative, 100.0), -100.0); // 微分項の飽和制限

  double control = Kp * error + Ki * integral + Kd * derivative;
  return control;
}

void tracer_task(intptr_t unused) {
  int error = g_colorSensor.getReflection() - targetBrightness;
  double control = calculatePid(error);

  g_right_motor.setPower(30 + control);
  g_left_motor.setPower(30 - control);
}
