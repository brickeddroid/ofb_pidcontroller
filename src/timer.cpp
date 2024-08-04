#include "timer.h"

template <typename T>
Timer<T>::Timer() : m_enabled(false) { }

template <typename T>
bool Timer<T>::isRunning() const { return m_enabled; }

template <typename T>
void Timer<T>::start(){
	m_enabled = true;
	t_start = millisT();
}

template <typename T>
void Timer<T>::stop(){
	m_enabled = false;
}

template <typename T>
bool Timer<T>::hasExpired(T intervalMs){
	T diff = millisT() - t_start;
	return m_enabled && (diff >= intervalMs);
}

template <typename T>
T Timer<T>::countdown(T intervalMs){
	if(!m_enabled) return 0;
	T maxT = -1;
	T t_stop = t_start + intervalMs;
	T cm = millisT();
	return (t_stop >= cm) ? t_stop - cm : maxT - cm + t_stop + 1;
}

template <typename T>
T Timer<T>::runtime()
{
	if(!m_enabled) return 0;
	T maxT = -1;
	T cm = millisT();
	return (cm >= t_start) ? cm - t_start : maxT - t_start + cm + 1;
}

template <typename T>
T Timer<T>::millisT()
{
#ifdef ARDUINO_ARCH_ESP32
	return esp_timer_get_time()/1000;
#else
    return millis();
#endif
}

template class Timer<uint16_t>;
template class Timer<uint32_t>;
template class Timer<uint64_t>;
