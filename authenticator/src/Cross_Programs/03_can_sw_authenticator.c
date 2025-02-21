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
#include <string.h>

#include <CAN_XR_Config.h>
#include <LED_Config.h>
#include <CAN_XR_PMA_GPIO.h>
#include <CAN_XR_PCS.h>
#include <CAN_XR_MAC.h>
#include <CAN_XR_Trace.h>
#include <stdbool.h>

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


int main(int argc, char *argv[])
{
    enable_leds();

    struct CAN_XR_MAC mac;
    struct CAN_XR_PCS pcs;
    struct CAN_XR_PMA pma;

    CAN_XR_PMA_GPIO_Init(&pma, GPIO_PRESCALER);
    CAN_XR_PCS_Init(&pcs, &pcs_parameters, &pma);

    /* TBD: To be replaced by implementation-specific initialization
       function when there's one. */
    CAN_XR_MAC_Common_Init(&mac, &pcs);

    /* Start the controller, feeding it with nodeclock indications. */
    SET_TRACE_TRESHOLD(3);
    CAN_XR_PMA_GPIO_NodeClock_Ind(&pma);

    return EXIT_SUCCESS;
}
