#include <Arduino.h>

#include <OFB_PIDController.h>

// Test values => current_input_value follows a simple sinus curve around the target value
//float angle_rad = 0.;


//float current_input_value = 25., target_value = 25.;
//float factor = 1.;
OFB::PIDController<float, uint8_t> pid_control_fp(0., 255-20, 1000);

//uint16_t current_input_value_ui16 = 25, target_value_ui16 = 25;
//uint16_t factor_ui16 = 1;
OFB::PIDController<int16_t, uint8_t> pid_control_i16(0, 255-50, 1000);

template <typename T_IN, typename T_OUT>
struct SensorMock {

    T_IN factor = 1;
    T_IN factor_diff;

    T_IN target_value;
    T_IN current_sensor_input;
    float angle_rad = 0.;



    void update(OFB::PIDController<T_IN, T_OUT>& pid_ctrl){
        if(!pid_ctrl.update(current_sensor_input)){
            return;
        }

        angle_rad += 0.01;
        if(angle_rad >= 2){
            angle_rad = 0.;
        }

        current_sensor_input = target_value + static_cast<T_IN>(2*static_cast<float>(factor) * sin(angle_rad * PI));

        if(factor > 0){
            factor -= factor_diff;
        }
        print(pid_ctrl);
    }

    void print(const OFB::PIDController<T_IN, T_OUT>& pid_ctrl){
        Serial.print(F("Sensor Input: "));
        Serial.print(current_sensor_input);
        Serial.print(F(" => PWM Output: "));
        Serial.println(pid_ctrl.current_output_value());
    }
};

SensorMock<float, uint8_t> sensor_mock_fp;
SensorMock<int16_t, uint8_t> sensor_mock_i16;


void setup()
{
  Serial.begin(115200);

  // Test code for demonstration
  sensor_mock_fp.factor       = 1.;
  sensor_mock_fp.factor_diff  = 0.001;
  sensor_mock_fp.target_value = 25.;

  sensor_mock_i16.factor       = 1000;
  sensor_mock_i16.factor_diff  = 1;
  sensor_mock_i16.target_value = 25000;

  // Initialization with Parameter Values
  pid_control_fp.set_kp(-3.5);
  pid_control_fp.set_ki(-0.5);
  pid_control_fp.set_kd(-0.1);

  pid_control_i16.set_kp(-4);
  pid_control_i16.set_ki(-1);
  pid_control_i16.set_kd(-1);

  // Set a target value regarding expected value range from  specific sensor (temp, humi, light,...)
  pid_control_fp.set_target_value(sensor_mock_fp.target_value);
  pid_control_i16.set_target_value(sensor_mock_i16.target_value);
}

void loop()
{
    sensor_mock_fp.update(pid_control_fp);
    sensor_mock_i16.update(pid_control_i16);
}
