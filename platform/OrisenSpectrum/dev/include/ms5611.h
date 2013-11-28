#ifndef _MS5611_H_
#define _MS5611_H_

extern const struct sensors_sensor ms5611_sensor;




#define RAW_MS5611_DATA 				0

#define MS5611_I2C_ADDR   			((0xEE)>>1)

#define MS5611_RESET						0x1E
#define MS5611_CONVERT_D1_256		0x40	// D1 = Digital pressure value
#define MS5611_CONVERT_D1_512		0x42
#define MS5611_CONVERT_D1_1024	0x44
#define MS5611_CONVERT_D1_2048	0x46
#define MS5611_CONVERT_D1_4096	0x48
#define MS5611_CONVERT_D2_256		0x50	// D2 = Digital temperature value
#define MS5611_CONVERT_D2_512		0x52
#define MS5611_CONVERT_D2_1024	0x54
#define MS5611_CONVERT_D2_2048	0x56
#define MS5611_CONVERT_D2_4096	0x58
#define MS5611_ADC_READ					0x00

#define MS5611_PROM_FACTORY			0xA0	// Factory data and the setup
#define MS5611_PROM_C1					0xA2	// Pressure sensitivity: SENS_T1
#define MS5611_PROM_C2					0xA4	// Pressure offset:	OFF_T1
#define MS5611_PROM_C3					0xA6	// Temperature coefficient of pressure sensitivity: TCS
#define MS5611_PROM_C4					0xA8	// Temperature coefficient of pressure offset: TCO
#define MS5611_PROM_C5					0xAA	// Reference temperature: T_REF
#define MS5611_PROM_C6					0xAC	// Temperature coefficient of the temperature: TEMPSENS
#define MS5611_PROM_SERIAL_CODE	0xAE	// Serial code and CRC

// Indices into config structure - correspond to those above
// Also used to obtain information from the value function.
//
#define MS5611_FACTORY					 0
#define MS5611_PSENS						 1
#define MS5611_POFF							 2
#define MS5611_TCS							 3
#define MS5611_TCO							 4
#define MS5611_TREF							 5
#define MS5611_TSENS						 6
#define MS5611_CRC							 7
//
// Sensor value types to be returned from value function - delays are given according to AN520
//
#define MS5611_LATCH_PRESSURE_256	     8	// This must be invoked at least 900us before reading
#define MS5611_LATCH_PRESSURE_512      9	// This must be invoked at least   3ms before reading
#define MS5611_LATCH_PRESSURE_1024    10	// This must be invoked at least   4ms before reading
#define MS5611_LATCH_PRESSURE_2048    11	// This must be invoked at least   6ms before reading
#define MS5611_LATCH_PRESSURE_4096    12	// This must be invoked at least  10ms before reading
#define MS5611_LATCH_TEMPERATURE_256  13	// This must be invoked at least 900us  before reading
#define MS5611_LATCH_TEMPERATURE_512  14	// This must be invoked at least   3ms before reading
#define MS5611_LATCH_TEMPERATURE_1024 15	// This must be invoked at least   4ms before reading
#define MS5611_LATCH_TEMPERATURE_2048 16	// This must be invoked at least   6ms before reading
#define MS5611_LATCH_TEMPERATURE_4096 17	// This must be invoked at least  10ms before reading
#define MS5611_PRESSURE_RAW			      18
#define MS5611_TEMPERATURE_RAW	      19


struct MS5611_data {
	uint32_t raw_pressure 	 ;	// Raw reading
	uint32_t raw_temperature ;	// Raw reading
} ;

// typedef struct MS5611_data MS5611_d

void MS5611_init(void);
void MS5611_start(void);
void MS5611_stop(void);
void MS5611_get_data(uint8_t *buffer);
void MS5611_decode(struct MS5611_data *const mag_data , uint8_t *buffer);

#endif /* MS5611_H_ */
