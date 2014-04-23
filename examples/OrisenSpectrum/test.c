/**
 * \file
 *         Test stuff for OrisenSpecrum
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/sensors.h"

#include "mpu9150.h"

int idx = 0;

/*---------------------------------------------------------------------------*/
PROCESS(test_process, "Test");
AUTOSTART_PROCESSES(&test_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();
  printf("clock second %i\n", CLOCK_SECOND);
  printf("Activating sensors....\n");
  SENSORS_ACTIVATE(mpu9150_sensor);

  printf("About to call mpu9150_who_am_i()...\n");
  uint8_t who_am_i_buffer[1];
  mpu9150_who_am_i(who_am_i_buffer);//(&who_am_i_buffer)
  // WhoAmI retuns the value we are interested in in bits [6:1],
  // so we need to zero out bits 7 and 0 and shift to left by one:
  uint8_t mask = 0b01111110;
  uint8_t who_am_i_trimmed = *who_am_i_buffer & mask;
  uint8_t who_am_i_shifted = lsr(who_am_i_trimmed, 0);
  printf("WHO_AM_I value: %i\n", who_am_i_shifted);
  
  //etimer_set(&et, 3*CLOCK_SECOND);
  //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  
  printf("clock second %i\n", CLOCK_SECOND);

  while (1) {
    printf("--------------------------- Iteration #%i ---------------------------\n", idx++);
    mpu9150_print_accel();
    resetFIFO();
    etimer_set(&et, CLOCK_SECOND * 0.2);//0.2
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    printf("\033[2J\033[1;1H");
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


/*
 * Function to return the WHO_AM_I value
 * of the mpu9150 chip.
*/
/*
void mpu9150_who_am_i(uint8_t *buffer) {
   //uint8_t set[] = {MPU9150_WHOAMI,0};

   uint8_t cmd = 0;

   cmd = MPU9150_WHOAMI;

   //i2c_transmitinit( MPU9150_I2C_ADDR, 1, &set );
   i2c_transmitinit( MPU9150_I2C_ADDR, 1, &cmd );
   while(!i2c_transferred());

   i2c_receiveinit( MPU9150_I2C_ADDR, 1, buffer );
   while(!i2c_transferred());
}
*/
