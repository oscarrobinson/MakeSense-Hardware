//project dependent variables and structures
#include "contiki-conf.h"
#include "sys/rtimer.h"

//sensor and network data structure
struct Sensor_Data {
	u16_t ADCValue;
	int16_t sht25_temp;
	int16_t RHData;
	int16_t bmp085_temp;
	int32_t bmp085_press;
};

struct sendpacket {
	unsigned long c_packetno;
	rtimer_clock_t send_time;
	struct Sensor_Data sd;
};
