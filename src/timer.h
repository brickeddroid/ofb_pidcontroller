#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

template <typename T>
class Timer {
private:
	T t_start;
	bool m_enabled;
protected:
	//virtual T milliseconds() = 0;

public:
	Timer();

	bool isRunning() const;
	void start();
	void stop();

	bool hasExpired(T intervalMs);
	T countdown(T intervalMs);
	T runtime();
	T millisT();
};

typedef Timer<uint16_t> Timer16;
typedef Timer<uint32_t> Timer32;
typedef Timer<uint64_t> Timer64;

#endif
