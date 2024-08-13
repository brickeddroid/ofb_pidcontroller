#ifndef OFB_PIDCONTROLLER_H
#define OFB_PIDCONTROLLER_H
#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32
#include <type_traits>
#endif

#include <OFB_Timer.h>

namespace OFB {
template <
    typename T_IN = float
#ifdef ARDUINO_ARCH_ESP32
    ,typename = typename std::enable_if<std::is_arithmetic<T_IN>::value, T_IN>::type
#endif
    ,typename T_OUT = int
#ifdef ARDUINO_ARCH_ESP32
    ,typename = typename std::enable_if<std::is_arithmetic<T_OUT>::value, T_OUT>::type
#endif
>
class PIDController {
private:
    T_IN m_kp;
    T_IN m_ki;
    T_IN m_kd;

    T_IN m_target_value;
    T_IN m_last_input;
    T_IN m_output_sum;

    T_OUT m_vmax;
    T_OUT m_output;


    Timer16 m_sample_timer;
    uint16_t m_sample_time;

public:
    PIDController(
        T_IN target_value,
        T_OUT vmax,
        uint16_t sample_time
    ) :
        m_kp(1),
        m_ki(1),
        m_kd(1),
        m_target_value(target_value),
        m_last_input(0),
        m_output_sum(0),
        m_vmax(vmax),
        m_output(0),
        m_sample_time(sample_time)
    {
        m_sample_timer.start();
    };

    bool update(T_IN input){
        if(!m_sample_timer.hasExpired(m_sample_time)) {
            return false;
        }

        m_sample_timer.start();

        T_IN error = m_target_value - input;
        T_IN dInput = (input - m_last_input);
        m_output_sum += (m_ki * error);

        if(m_output_sum >= m_vmax) m_output_sum = m_vmax;
        else if(m_output_sum < 0) m_output_sum = 0;

        m_output = m_kp * error;

        m_output += m_output_sum - m_kd * dInput;

        if(m_output >= m_vmax) m_output = m_vmax;
        else if(m_output <= 0) m_output = 0;

        m_last_input = input;

        return true;
    };

    inline void set_sample_time(uint16_t stime){ m_sample_time = stime; }

    inline T_IN kp() const { return m_kp; }
    inline T_IN ki() const { return m_ki; }
    inline T_IN kd() const { return m_kd; }

    inline void set_kp(T_IN kp){ m_kp = kp; }
    inline void set_ki(T_IN ki){ m_ki = ki; }
    inline void set_kd(T_IN kd){ m_kd = kd; }

    inline T_IN target_value() const { return m_target_value; }
    inline void set_target_value(T_IN target_value) { m_target_value = target_value; }

    inline T_OUT current_output_value() const { return m_output; }
};

} // end namespace OFB
#endif
