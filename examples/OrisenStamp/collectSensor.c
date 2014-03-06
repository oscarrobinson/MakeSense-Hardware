/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: example-collect.c,v 1.16 2011/01/10 15:11:44 adamdunkels Exp $
 */

/**
 * \file
 *         Example of how the collect primitive works.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "contiki-uart.h"
#include <mc1322x.h>
#include <board.h>
#include <stdlib.h>
#include <string.h>


#include "net/netstack.h"

#include <stdio.h>

static struct collect_conn tc;

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}

char * ID;

char * ontologyID = "JayISCOOL";


/*---------------------------------------------------------------------------*/
PROCESS(example_collect_process, "Test collect process");
PROCESS(uart_process, "UART Reader");
AUTOSTART_PROCESSES(&uart_process);


static char * idAsString(){
   rimeaddr_t ID = rimeaddr_node_addr;
   int counter = 0;
   char * idString = malloc(17*sizeof(char));
   for(counter = 0;counter<8;counter++){
      char * temp = malloc(3*sizeof(char));
      sprintf(temp, "%02X\0", ID.u8[counter]); 
      strcat(idString, temp);
      free(temp);
      printf("%02X\n",ID.u8[counter]);
   }
   idString[16] = '\0';
   return idString;
}
/*---------------------------------------------------------------------------*/
static void
recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  printf("Sink got message from %d.%d, seqno %d, hops %d: len %d '%s'\n",
	 originator->u8[0], originator->u8[1],
	 seqno, hops,
	 packetbuf_datalen(),
	 (char *)packetbuf_dataptr());
}
/*---------------------------------------------------------------------------*/
static const struct collect_callbacks callbacks = { recv };
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_collect_process, ev, data)
{
  static struct etimer periodic;
  static struct etimer et;
  
  PROCESS_BEGIN();
  //set_power(0x01);


      static rimeaddr_t oldparent;
      const rimeaddr_t *parent;

   int dataLength = 0;
   dataLength = strlen(data);
  
   //get the timestamp
   unsigned long timestamp;
   timestamp = clock_seconds();
   //get length of timestamp
   int timestampLen = 0;
   int temp = (int)timestamp;
   while (temp>0){
      temp/=10;
      timestampLen++;
   }

      
   //convert id to string
   int idStringLen = 30;
   char * idString = malloc(idStringLen * sizeof(char));
   sprintf(idString,"%s",ID);

   //get length of ontologyID
   int ontologyIDLen = strlen(ontologyID);

   //prepare the string to send
   int numberOfExtraChars = 9;
   char * stringToSend = malloc((timestampLen + dataLength + idStringLen + ontologyIDLen +  numberOfExtraChars)*sizeof(char));

   sprintf(stringToSend,"%s X %s X %lu X %s", idString,data, timestamp, ontologyID);
   

     printf("%s \n", stringToSend);
      

      printf("Sending\n");
      packetbuf_clear();
      packetbuf_set_datalen(sprintf(packetbuf_dataptr(),
				  "%s", stringToSend) + 1);
      collect_send(&tc, 15);
      printf("Sent\n");
      FLASH_LED(LEDS_BLUE);

      parent = collect_parent(&tc);
      if(!rimeaddr_cmp(parent, &oldparent)) {
        if(!rimeaddr_cmp(&oldparent, &rimeaddr_null)) {
          printf("#L %d 0\n", oldparent.u8[0]);
        }
        if(!rimeaddr_cmp(parent, &rimeaddr_null)) {
          printf("#L %d 1\n", parent->u8[0]);
        }
        rimeaddr_copy(&oldparent, parent);
      }
    
  
  
   free(data);
   free(stringToSend);
   free(idString);

  PROCESS_END();
}


PROCESS_THREAD(uart_process, ev, data)
{

  PROCESS_BEGIN();
  ID = idAsString();
  static struct etimer et;

  leds_off(LEDS_ALL);

  // Set UART2 to 115200 baud
  uart2_init(INC, MOD, SAMP);
  



  SENSORS_ACTIVATE(button_sensor);
  
  collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);
  printf("Opened connection\n");
  
  etimer_set(&et, 5 * CLOCK_SECOND);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
  static int size = 30;
  char *  dataRead = malloc(size * sizeof(char));
  static int counter = 0;
  while(1) {

    /* Wait for button click before sending the first message. */
    //PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
    char ch = uart2_getc();
    printf("Got char: %c  Counter: %d\n", ch, counter);
    //etimer_set(&et, 1 * CLOCK_SECOND);
    //PROCESS_WAIT_UNTIL(etimer_expired(&et));
    if (ch=='\n'){
         //the character before \n is a \r, we don't want that so we overwrite it with a \0 which we do need
         dataRead[counter-1] = '\0';
         //copy data into new string
         int i=0;
         char * dataString = malloc((counter) * sizeof(char));
         for(i=0;i<(counter);i++){
  	         dataString[i]=*((char*)dataRead+i);
         }
         printf("reading data: %s\n", dataString);

      	 printf("Button clicked\n");

    
        process_start(&example_collect_process, dataString);
                
         free(dataRead);

         etimer_set(&et, 1 * CLOCK_SECOND);
         PROCESS_WAIT_UNTIL(etimer_expired(&et));

        dataRead = malloc(size * sizeof(char));	
        counter=0;
	
    }
    else{
      dataRead[counter]=ch;
      counter++;
    }
  }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
