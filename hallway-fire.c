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
 */

/**
 * \file
 *         Best-effort single-hop unicast example
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "node-id.h"
#include "dev/button-sensor.h"

#include "dev/leds.h"

#include <stdio.h>

#define PATH 0
#define FIRE 1
#define EXIT 2

/*---------------------------------------------------------------------------*/
PROCESS(hallway_lights, "Hallway test");
AUTOSTART_PROCESSES(&hallway_lights);
static struct ctimer health_timer, alarm_timer;
static int direction = 0;
int greenStatus = 0;
int redStatus = 0;
int alarmed = 0;
int nodeType = {EXIT, PATH, EXIT};
//static int layout = [-1, 0, 1, 2];
/*---------------------------------------------------------------------------*/

static void flashRed () {
  redStatus = !redStatus;
  if (redStatus) leds_on(LEDS_RED);
  else leds_off(LEDS_RED);
  ctimer_set(&alarm_timer, 0.5 * CLOCK_SECOND, &flashRed, NULL);
}



static void flashGreen() {
  greenStatus = !greenStatus;
  if (greenStatus) leds_on(LEDS_GREEN);
  else leds_off(LEDS_GREEN);
  ctimer_set(&health_timer, 1.0 * CLOCK_SECOND, &flashGreen, NULL);
}


static void soundAlarm() {
  if (!alarmed) {
    flashRed();
    alarmed = 1;
  }
}
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
  printf("unicast message received from %d.%d - %d (%d)\n",
 		from->u8[0], from->u8[1], ((uint8_t*)packetbuf_dataptr())[0], packetbuf_datalen());
  if (((uint8_t*)packetbuf_dataptr())[0] == FIRE) {
  	if (from->u8[0] + 1 == linkaddr_node_addr.u8[0]) {
  		direction = 1;
      leds_on(LEDS_GREEN);
  	} else if (from->u8[0] - 1 == linkaddr_node_addr.u8[0]) {
  		direction = -1;
      leds_on(LEDS_BLUE);
  	}
     /* Set a timer for when to turn off */
     soundAlarm();
     send_alert();
   }
}


static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;

void send_alert() {
  /* Alert neighbours behind and ahead */
  linkaddr_t addr;
  printf("my addr: %d - %d\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
  uint8_t data[] = {FIRE};
  packetbuf_copyfrom(data, 1);
  addr.u8[0] = linkaddr_node_addr.u8[0] + 1;
  addr.u8[1] = 0;
  int s = unicast_send(&uc, &addr);
  printf("Sent1: %d - %d\n", s, addr.u8[0]);

  // addr.u8[0] = linkaddr_node_addr.u8[0] - 1;
  // addr.u8[1] = 0;
  // packetbuf_copyfrom(data, 1);
  // s = unicast_send(&uc, &addr);
  // printf("Sent2: %d - %d\n", s, addr.u8[0]);

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hallway_lights, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor);

  unicast_open(&uc, 146, &unicast_callbacks);
  //flashGreen();
  while(1) {

    linkaddr_t addr;
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event &&
			     data == &button_sensor);
    //ctimer_stop(&health_timer);
    leds_on(LEDS_GREEN);
    /* Flash my siren/light */
    soundAlarm();

    /* Alert neighbours behind and ahead */
    printf("my addr: %d - %d\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    uint8_t data[] = {FIRE};
    packetbuf_copyfrom(data, 1);
    addr.u8[0] = linkaddr_node_addr.u8[0] + 1;
    addr.u8[1] = 0;
    int s = unicast_send(&uc, &addr);
    addr.u8[0] = linkaddr_node_addr.u8[0] - 1;
    addr.u8[1] = 0;
    packetbuf_copyfrom(data, 1);
    s = unicast_send(&uc, &addr);
    printf("Sent1: %d - %d\n", s, addr.u8[0]);

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
