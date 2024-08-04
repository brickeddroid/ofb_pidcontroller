#include <Arduino.h>

#include "pid-controller.h"

// Test values => current_input_value follows a simple sinus curve around the target value
float angle_rad = 0.;


float current_input_value = 25., target_value = 25.;
float factor = 1.;
PidController<float, uint8_t> pid_control(target_value, 255-20, 1000);

uint16_t current_input_value_ui16 = 25, target_value_ui16 = 25;
uint16_t factor_ui16 = 1;
PidController<uint16_t, uint8_t> pid_control_ui16(target_value_ui16, 255-50, 1000);


template <typename T>
void update(PidController<T, uint8_t>& pid_ctrl, T current_sensor_input){
  if(!pid_ctrl.update(current_sensor_input)){
    return;
  }
  
  angle_rad += 0.01;
  if(angle_rad >= 2){
    angle_rad = 0.;
  }
  Serial.print(F("Sensor Input: "));
  Serial.print(current_input_value);
  Serial.print(F(" => PWM Output: "));
  Serial.println(pid_ctrl.current_output_value());

}

void setup()
{
  Serial.begin(115200);
  pid_control.set_kp(-3.5);
  pid_control.set_ki(-0.5);
  pid_control.set_kd(-0.1);
}

void loop()
{
  if(pid_control.update(current_input_value)){

    // Only for simulation and testing purposes
    angle_rad += 0.01;
    if(angle_rad >= 2){
      angle_rad = 0.;
    }
  
#ifdef ARDUINO_ARCH_ESP32
    Serial.printf("Sensor Input: %.2f => PWM Output: %d\r\n", current_input_value, pid_control.current_output_value());
#else
    Serial.print(F("Sensor Input: "));
    Serial.print(current_input_value);
    Serial.print(F(" => PWM Output: "));
    Serial.println(pid_control.current_output_value());
#endif

    // Fancy function to create a sensor value by hand
    current_input_value = target_value + 2*factor * sin(angle_rad * PI);
    current_input_value_2 = target_value_2 + 2*factor_2 * sin(angle_rad * PI);

    // Getting closer and closer to the target value
    if(factor > 0){
      factor -= 0.001;
    }

    if(factor_2 > 0){
      --factor_2;
    }
  }
}
