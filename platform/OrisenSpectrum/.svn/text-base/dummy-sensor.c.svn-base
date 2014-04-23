#include "dummy-sensor.h"

const struct sensors_sensor dummy_sensor;

static int
value(int type)
{
	return 0;
}

static int
configure(int type, int c)
{
	return 0;
}

static int
status(int type)
{
	return 0;
}

SENSORS_SENSOR(dummy_sensor, DUMMY_SENSOR, value, configure, status);

