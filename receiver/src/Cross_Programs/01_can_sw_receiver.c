/*+
    SDCC - A Software-Defined CAN Controller
    Copyright (C) 2018 National Research Council of Italy and
      University of Luxembourg

    Authors: Ivan Cibrario Bertolotti and Tingting Hu

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2
    as published by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    The use of the program may be restricted in certain countries by
    intellectual property rights owned by Bosch.  For more information, see:

    http://www.bosch-semiconductors.com/ip-modules/can-ip-modules/can-protocol/
+*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <CAN_XR_Config.h>
#include <LED_Config.h>
#include <CAN_XR_PMA_GPIO.h>
#include <CAN_XR_PCS.h>
#include <CAN_XR_MAC.h>
#include <CAN_XR_Trace.h>
//#include <bpmac.h>
#include "../../lib/bpmac/bpmac.h"
#include <stdbool.h>
#include <string.h>


#define configCPU_CLOCK_HZ 96000000    // 96 MHz is used clock speed at Mbed development board

/* 8 quanta per bit, sampling point between quantum 5 and 6, (assuming
   the first quantum is quantum 0).  This matches the configuration of
   the hardware CAN controller performed by CAN_XR_CAN_Driver.c.
*/
const struct CAN_XR_PCS_Bit_Time_Parameters pcs_parameters = {
    .prescaler_m = 1,
    .sync_seg = 1, /* Always this way */
    .prop_seg = 3,
    .phase_seg1 = 2,
    .phase_seg2 = 2,
    .sjw = 1
};

/* The GPIO PMA takes its timing reference from Timer 0, clocked at
   the CCLK frequency, configCPU_CLOCK_HZ.  The prescaler value must
   be calculated in the same way as a normal CAN controller, starting
   from that frequency.
*/
#define GPIO_BIT_RATE CAN_XR_BIT_RATE
#define GPIO_NODECLOCK_PER_BIT (pcs_parameters.sync_seg + pcs_parameters.prop_seg + pcs_parameters.phase_seg1 + pcs_parameters.phase_seg2)
#define GPIO_PRESCALER configCPU_CLOCK_HZ/(GPIO_BIT_RATE*GPIO_NODECLOCK_PER_BIT)

struct CAN_XR_MAC mac;
struct CAN_XR_PCS pcs;
struct CAN_XR_PMA pma;
// MAC stuff
bpmac_ctx_t ctx_grp;
uint64_t grp_nonce[2] = {0, 0};
uint8_t grp_key[] = {0xFF, 0x00,0xFF, 0x00,0xFF, 0x00,0xFF, 0x00,0xFF, 0x00,0xFF, 0x00,0xFF, 0x00,0xFF, 0x00};
uint8_t grp_key_nonce[] = {0x00, 0xFF,0x00, 0xFF,0x00, 0xFF,0x00, 0xFF,0x00, 0xFF,0x00, 0xFF,0x00, 0xFF,0x00, 0xFF};

uint16_t correct = 0;
uint16_t incorrect = 0;
uint16_t signaling_state = 0;
uint8_t unauth_cnt = 0;
int signal_cnt = 5;
int msg_limit = 10005;
int msg_cnt = 0;

void dummy_data_ind(
    struct CAN_XR_LLC *llc, unsigned long ts, uint32_t identifier,
    enum CAN_XR_Format format, int dlc, uint8_t *data)
{
    switch (identifier) {
        case 555:   /* (1) 10k message signal */
            signaling_state = 383;
            break;

        case 279:   /* (3) transmission attempts */
            signaling_state = 525;
            break;

        case 200:   /* got new nonce value from sender */
            /* set new nonce */

            /* MULTI-RECEIVER SETUP: backup nonce in case other controller has triggered a nonce resynchronization */
//            uint64_t backup_nonce[2] = {grp_nonce[0], grp_nonce[1]};

            memcpy(((uint8_t *) &grp_nonce[1]) + 3, data, 5);
            grp_nonce[0] = 0;

            /* validate MAC */
            uint8_t grp_mac[16] = {0};
            bpmac_pre(&ctx_grp, (uint8_t *) grp_nonce, (char *) grp_mac);

            for(int8_t i = 10; i >= 0; i--) {
                bpmac_update(&ctx_grp, (identifier & (1 << i)), (char *) grp_mac);
            }
            bpmac_sign(&ctx_grp, (char *) data, 5, (char *) grp_mac);

            if (!(data[5] == grp_mac[1] && data[6] == grp_mac[2] && data[7] == grp_mac[3]))
            {
                /* if MAC incorrect */
                led_on(led2);
                signaling_state = 384;

                /* MULTI-RECEIVER SETUP: reset nonce to old value and return to receiving state afterwards */
//                memcpy(&grp_mac, &backup_nonce, 16);
//                signaling_state = 0
            }
            else
            {
                led_on(led4);
                if (++grp_nonce[0] == 0)
                {
                    grp_nonce[1]++;
                }
                unauth_cnt = 0;
                signaling_state = 0;

            }
            break;

        default:    /* check for CAIBA authenticated message and validate*/
            if (signaling_state == 0 && identifier <= 256)
            {
                led_off(led2);
                led_off(led4);

                uint8_t grp_mac[16] = {0};

                bpmac_pre(&ctx_grp, (uint8_t *) grp_nonce, (char *) grp_mac);

                /* msg id is covered by MAC */
                for(int8_t i = 10; i >= 0; i--) {
                    bpmac_update(&ctx_grp, (identifier & (1 << i)), (char *) grp_mac);
                }

                bpmac_sign(&ctx_grp, (char *) data, dlc - 3, (char *) grp_mac);



                if (++grp_nonce[0] == 0)
                {
                    grp_nonce[1]++;
                }

                /* VALIDATION */
                static uint8_t on = 0;

                /* correct MAC received */
                if (data[dlc - 3] == grp_mac[1] && data[dlc - 2] == grp_mac[2] && data[dlc - 1] == grp_mac[3]) {
                    if (!on) {
                        led_on(led1);
                        on = 1;
                    } else {
                        led_off(led1);
                        on = 0;
                    }
                    led_off(led4);
                    led_off(led2);
                    correct += 1;
                    unauth_cnt = 0;
                }
                /* MAC incorrect */
                else
                {
                    unauth_cnt++;
                    reset_leds();
                    led_on(led4);
                    led_on(led2);
                    if (unauth_cnt == 5)    /* trigger nonce reset */
                    {
                        signaling_state = 384;
                    }
                    incorrect += 1;
                }

                if (++msg_cnt >= msg_limit) {
                    signaling_state = 383;
                }
            }
        
    }
}

void app_nodeclock_ind(
        struct CAN_XR_PCS *pcs, int bus_level)
{

    static int msg_intervals = 0;

    if (msg_intervals++ % 6000 == 0) {      // for one message every ~25 ms @ 40 kbs
        switch (signaling_state) {
            case 384:   /* trigger nonce reset on all nodes */
                led_on(led3);   /* nonce reset lec */
                led_off(led2);  /* reset wrong MAC leds */
                led_off(led4);

                CAN_XR_MAC_Data_Req(&mac, 384, CAN_XR_FORMAT_CBFF, 1, (uint8_t *) unauth_cnt);
                signaling_state = 999;
                break;

            case 383:   /* (2) answer to 10k message signal -> send #correct MACs received */
                CAN_XR_MAC_Data_Req(&mac, 383, CAN_XR_FORMAT_CBFF, 2, (uint8_t *) &correct);
                signaling_state = 0;
                break;

            case 525:   /* (4) answer to #transmission attempts -> send #incorrect MACs received */
                CAN_XR_MAC_Data_Req(&mac, 525, CAN_XR_FORMAT_CBFF, 2, (uint8_t *) &incorrect);
                led_set_all();
                if (--signal_cnt == 0) {    /* repeat 5 times */
                    signaling_state = 418;
                } else {
                    signaling_state = 383;
                }
                break;

            case 418:   /* (5) signalize continue */
                msg_cnt = 0;
                signaling_state = 0;
                signal_cnt = 5;
                correct = 0;
                incorrect = 0;
                uint8_t data = 0xFF;
                CAN_XR_MAC_Data_Req(&mac, 418, CAN_XR_FORMAT_CBFF, 1, (uint8_t *) &data);
                reset_leds();
                break;

            default:
                // noop
                break;
        }
    }
}

int main(int argc, char *argv[])
{
    enable_leds();

    bpmac_init((char *) grp_key, (char *) grp_key_nonce, 8, &ctx_grp);

    CAN_XR_PMA_GPIO_Init(&pma, GPIO_PRESCALER);
    CAN_XR_PCS_Init(&pcs, &pcs_parameters, &pma);

    /* TBD: To be replaced by implementation-specific initialization
       function when there's one. */
    CAN_XR_MAC_Common_Init(&mac, &pcs);

    /* Register app_nodeclock_ind to trigger the transmission */
    CAN_XR_PMA_GPIO_Set_App_NodeClock_Ind(&pma, app_nodeclock_ind);

    /* Register a dummy data_ind primitive in 'mac'. */
    CAN_XR_MAC_Set_Data_Ind(&mac, dummy_data_ind);

    /* Start the controller, feeding it with nodeclock indications. */
    SET_TRACE_TRESHOLD(3);
    CAN_XR_PMA_GPIO_NodeClock_Ind(&pma);

    return EXIT_SUCCESS;
}
