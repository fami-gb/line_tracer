#include <iostream>
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
#include <serial/serial.h>
#include <serial/newlib.h>
#include <syssvc/serial.h>

#include <libcpp/spike/Port.h>

extern "C" {
  void __attribute__((weak)) _fini() {}
  void* __dso_handle __attribute__((weak)) = (void*)-1;
}

using namespace spikeapi;
IMU g_imu;
Display g_display;
ForceSensor g_forceSensor(EPort::PORT_D);
ColorSensor g_colorSensor(EPort::PORT_E);
ColorSensor::HSV g_hsv;
Button g_button;
Clock g_clock;
FILE *fp;
int targetBrightness = 0;

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

void ColorJugde(ColorSensor::HSV &hsv) {

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
    }

    if (blackCalibrated) {
      BlackBrightness = g_colorSensor.getReflection();
      g_display.showNumber(BlackBrightness);

      if (g_button.isRightPressed()) {
        fprintf(fp, "黒の輝度測定が終了しました。\n");
        break;
      }
    }
  }
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
  sta_cyc(TRACER_TASK_CYC); 
  ext_tsk();
}

void tracer_task(intptr_t unused) {

}
