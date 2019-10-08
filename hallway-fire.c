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
PROCESS(fire_alarm, "Fire Alam");
AUTOSTART_PROCESSES(&fire_alarm);
static struct ctimer health_timer, alarm_timer;
static int direction = 0;
int last_addr = -1;
int dist = 255;
int greenStatus = 0;
int redStatus = 0;
int alarmed = 0;
int nodeType = {EXIT, PATH, EXIT};
int exitNodes[2] = {13, 1};
int isExit = 0;
int isFire = 0;
int map[][3] = {
  {2, 25, -1}, //1
  {1, 3, -1}, // 2
  {2, 4, -1}, // 3
  {3, 5, -1}, // 4
  {4, 6, -1}, //5
  {5, 7, 8},
  {6, 8, 20},
  {7, 9, -1},
  {8, 10, -1},
  {9, 11, -1},
  {10, 12, -1},
  {11, 14, -1},// 12
  {14, -1, -1}, // 13
  {12, 13, 15}, // 14
  {14, 16, -1},
  {15, 17, -1},
  {16, 18, -1},
  {17, 19, -1},
  {18, 20, 21}, //19
  {19, 7, -1}, //20
  {22, 19, -1},
  {21, 23, -1},
  {22, 24, -1},
  {23, 25, -1},
  {24, 1, -1}
};
//static int layout = [-1, 0, 1, 2];
/*---------------------------------------------------------------------------*/

static void checkIsExit() {
  int len = sizeof(exitNodes) / sizeof(int);
  int i;
  for (i = 0; i < len; i++) {
    if (linkaddr_node_addr.u8[0] == exitNodes[i]) {
      printf("exit node (%d)\n",linkaddr_node_addr.u8[0]);
      isExit = 1;
      dist = 0;
    }
  }
}
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
recv_uc(struct unicast_conn *c, const linkaddr_t *from) {
  last_addr = from->u8[0];
  printf("unicast message received from %d.%d - %d (%d)\n",
 		from->u8[0], from->u8[1], ((uint8_t*)packetbuf_dataptr())[0], packetbuf_datalen());
  if (from->u8[0] + 1 == linkaddr_node_addr.u8[0]) {
      direction = 1;
      leds_on(LEDS_GREEN);
  } else if (from->u8[0] - 1 == linkaddr_node_addr.u8[0]) {
    direction = -1;
    leds_on(LEDS_BLUE);
  }

  if (((uint8_t*)packetbuf_dataptr())[0] == FIRE) {

     /* Set a timer for when to turn off */
     soundAlarm();
     send_alert(((uint8_t*)packetbuf_dataptr())[1]);
     if (isExit && !isFire) {
      last_addr = -1;
      send_exit(from->u8[0], dist);
      // send_exit(linkaddr_node_addr.u8[0] + 1, dist);
     }
   }
   else if (((uint8_t*)packetbuf_dataptr())[0] == EXIT) {

    uint8_t path = ((uint8_t*)packetbuf_dataptr())[1] + 1;
    if (isFire == 0) {
      if (path < dist) {
        dist = path;
        printf("from: %d - direction: %d - next: %d\n", from->u8[0], direction, linkaddr_node_addr.u8[0] - direction);
        send_exit(linkaddr_node_addr.u8[0] + direction, dist);
        printf("@%d@\n", from->u8[0]);

      }
      else if (path > dist + 1){
        last_addr = -1;
        printf("got (%d) sending better dist next: %d\n", ((uint8_t*)packetbuf_dataptr())[1] + 1, dist);
        send_exit(linkaddr_node_addr.u8[0] + direction, dist);

      }
    }
    else {
      printf("Stopping exit message from my addr: %d - closest exit in %d hops\n", linkaddr_node_addr.u8[0], path);
      printf("@%d@\n", from->u8[0]);

    }
   }
}


static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;

void send_to_neighbours(uint8_t data[], int len){
  int *neighbours = map[linkaddr_node_addr.u8[0] -1];
  int i = 0;
  for (i = 0; i < 3 && neighbours[i] != -1; i++){
    if (neighbours[i] == last_addr) continue;
    linkaddr_t addr;
    packetbuf_copyfrom(data, len);
    addr.u8[0] = neighbours[i];
    addr.u8[1] = 0;
    int s = unicast_send(&uc, &addr);
    printf("sent %d byte(s) to %d\n", s, addr.u8[0]);

  }
}
void send_exit(uint8_t rcv, uint8_t dist) {
  /* Alert neighbours behind and ahead */
  linkaddr_t addr;
  printf("Exit from my addr: %d - in %d hops\n", linkaddr_node_addr.u8[0], dist);
  uint8_t data[] = {EXIT, dist};
  // packetbuf_copyfrom(data, 2);
  addr.u8[0] = rcv;
  addr.u8[1] = 0;
  // int s = unicast_send(&uc, &addr);
  send_to_neighbours(data, 2);
  printf("Forwarded Exit:  \n");
}
void send_alert(uint8_t ttl) {
  /* Alert neighbours behind and ahead */
  if (ttl == 0){
    printf("TTL expired");
    return;
  }
  linkaddr_t addr;
  printf("Fordwarding Fire - my addr: %d - %d\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
  uint8_t data[] = {FIRE, ttl - 1};
  // packetbuf_copyfrom(data, 1);
  // addr.u8[0] = linkaddr_node_addr.u8[0] + 1;
  // addr.u8[1] = 0;
  // int s = unicast_send(&uc, &addr);
  send_to_neighbours(data, 2);
  printf("Forwarded Alert ttl(%d):\n", ttl -1);

  // addr.u8[0] = linkaddr_node_addr.u8[0] - 1;
  // addr.u8[1] = 0;
  // packetbuf_copyfrom(data, 1);
  // s = unicast_send(&uc, &addr);
  // printf("Sent2: %d - %d\n", s, addr.u8[0]);

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(fire_alarm, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor);

  unicast_open(&uc, 146, &unicast_callbacks);
  //flashGreen();
  checkIsExit();
  while(1) {

    linkaddr_t addr;
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event &&
			     data == &button_sensor);
    //ctimer_stop(&health_timer);
    leds_on(LEDS_GREEN);
    /* Flash my siren/light */
    soundAlarm();

    /* Alert neighbours behind and ahead */
    printf("Fire at my addr: %d - %d\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    uint8_t data[] = {FIRE, 16};
    send_to_neighbours(data, 2);
    // packetbuf_copyfrom(data, 1);
    // addr.u8[0] = linkaddr_node_addr.u8[0] + 1;
    // addr.u8[1] = 0;
    // int s = unicast_send(&uc, &addr);
    // printf("Sent alert: %d - %d\n", s, addr.u8[0]);
    // addr.u8[0] = linkaddr_node_addr.u8[0] - 1;
    // addr.u8[1] = 0;
    // packetbuf_copyfrom(data, 1);
    // s = unicast_send(&uc, &addr);
    // printf("Sent alert: %d - %d\n", s, addr.u8[0]);
    printf("Sent fire alert\n");

    isFire = 1;

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
