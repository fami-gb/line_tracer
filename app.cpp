#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdio.h>
#include <string>

#include "kernel_cfg.h"
#include "app.h"
#include "Pid.hpp"

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
  Motor g_left_motor(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE);
  ForceSensor g_forceSensor(EPort::PORT_D);
  ColorSensor g_colorSensor(EPort::PORT_E);
  Button g_button;
  Clock g_clock;
  FILE *fp;
  int targetBrightness = 0;
  std::chrono::milliseconds previousTime = std::chrono::milliseconds(0);
  Pid g_pid(targetBrightness, 0.8, 0.01, 0.1); // PID制御のパラメータを設定
}

uint64_t msNow() { return g_clock.now() / 1000; }

void SerialCalibration() {
    // Bluetooth接続が確立されるまで待機
    bool connected = false;
    ER err;
    while (!connected) {
        err = serial_opn_por(SIO_BLUETOOTH_PORTID);
        fp = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);
        if (err == E_OK && fp != nullptr) connected = true;
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
  while(1) {
    if (g_forceSensor.isPressed(0.5f)) {
      while (g_forceSensor.isTouched()) { }
      break;
    }
  } 
  fprintf(fp, "フォースセンサが押されました。\n");
  g_clock.reset();
  sta_cyc(TRACER_TASK_CYC); 
  act_tsk(CALIBRATION_TASK);
  ext_tsk();
}

int clamp(int value, int min_val=-100, int max_val=100) {
  return std::max(std::min(value, max_val), min_val);
}

void tracer_task(intptr_t unused) {
  if (g_forceSensor.isPressed(0.5f)) {
    stp_cyc(TRACER_TASK_CYC);

    // pidゲインの再設定用タスク
    act_tsk(CALIBRATION_TASK);
    g_pid.Reset();

    ext_tsk();
    return;
  }

  std::chrono::milliseconds now = std::chrono::milliseconds(msNow());
  double dt = std::chrono::duration<double>(now - previousTime).count(); // 経過時間を秒に変換
  previousTime = now;

  int error = g_colorSensor.getReflection() - targetBrightness;
  int control = g_pid.calculate(error, dt);

  g_right_motor.setPower(clamp(30 + control));
  g_left_motor.setPower(clamp(30 - control));

  ext_tsk();
}

/* 再キャリブレーション用タスク（act_tskで起動される） */
void calibration_task(intptr_t unused) {
  fprintf(fp, "\n===== 再キャリブレーション =====\n");

  // 同じ場所で走らせるなら不要。(輝度の再設定が必要になったときにコメントアウトを外す)
  // BrightnessCalibration();

  
  /*
   * struct PidGain {
   *    double p,i,d;
   * }
   *
   * PidgGain getGain(){}
   *
   */
  PidGain pidGain = g_pid.getPidGain();
  char input[256];

  fprintf(fp, "pidゲインの再設定を行います。\n");
  fprintf(fp, "Pゲインを設定してください。(変更しない場合はEnter)\n");
  if (fgets(input, sizeof(input), fp) != nullptr) {
    if (input[0] != '\n' && input[0] != '\r') {
      pidGain.kp_ = atof(input);
    }
  }
  fprintf(fp, "Iゲインを設定してください。(変更しない場合はEnter)\n");
  if (fgets(input, sizeof(input), fp) != nullptr) {
    if (input[0] != '\n' && input[0] != '\r') {
      pidGain.ki_ = atof(input);
    }
  }
  fprintf(fp, "Dゲインを設定してください。(変更しない場合はEnter)\n");
  if (fgets(input, sizeof(input), fp) != nullptr) {
    if (input[0] != '\n' && input[0] != '\r') {
      pidGain.kd_ = atof(input);
    }
  }

  g_pid.setPidGain(pidGain.kp_, pidGain.ki_, pidGain.kd_);

  // フォースセンサのボタンが押されるまで待機
  fprintf(fp, "フォースセンサを押してください\n");
  while(1) {
    if (g_forceSensor.isPressed(0.5f)) {
      while (g_forceSensor.isTouched()) { }
      break;
    }
  }
  fprintf(fp, "フォースセンサが押されました。\n");
  g_clock.reset();
  sta_cyc(TRACER_TASK_CYC);
  ext_tsk();
}
