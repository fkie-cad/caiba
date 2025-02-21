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
/* This file implements the functions common to all MACs, for
   instance, setters and primitive invocation functions.

   Full TRACE at level 2.
   Errors are emitted at TRACE level 9.
*/

#include <stdlib.h>
#include <string.h>
#include <LED_Config.h>
#include <CAN_XR_PCS.h>
#include <CAN_XR_MAC.h>
#include <CAN_XR_Trace.h>
//#include <bpmac.h>
#include "../../lib/bpmac/bpmac.h"



#define shift_in(v, b) (((v) << 1) | ((b) & 0x1))

/* Prepare v, which is n_bits wide (<= 32) for MSb-first shifting.
   Return the prepared value as uint32_t.
*/
#define shift_prepare(v, n_bits) ((uint32_t)(v) << (32 - (n_bits)))

/* Store into b the bit of v to be transmitted.  Shift v to put the
   next bit into position.  v is assumed to be uint32_t.
*/
#define shift_out(b, v)				\
    do {					\
	b = ((v) & 0x80000000) ? 1 : 0;		\
	v <<= 1;				\
    } while(0)

/* MAC_Data.Request primitive invoked by upper later (typically LLC) to
   request the transmission of a frame.
*/
static void mac_data_req(
    struct CAN_XR_MAC *mac,
    uint32_t identifier, enum CAN_XR_Format format, int dlc, uint8_t *data)
{
    TRACE(2, "MAC Common::mac_data_req(%lu, ...)", (unsigned long)identifier);

    /* Check if there is a pending transmission request already, of
       any kind.  This would be an LLC handshake error.
    */
    if(mac->state.data_req_pending)
    {
        if(mac->primitives.data_conf)
        {
            mac->primitives.data_conf(mac->llc, 0,
                                      identifier, CAN_XR_MAC_TX_STATUS_NO_SUCCESS);
        }
    }

    else
    {
        /* Check format, complain immediately if it is unsupported */
        switch(format)
        {
        case CAN_XR_FORMAT_CBFF:
            /* Save arguments in MAC state for later use. */
            mac->state.tx_identifier = identifier;
            mac->state.tx_format = format; /* TBD: We'll support more */
            mac->state.tx_dlc = dlc;
            /* Clear tx_data completely, then fill the right amount */
            memset(mac->state.tx_data, 0, sizeof(mac->state.tx_data));
            memcpy(mac->state.tx_data, data, dlc);
            mac->state.data_req_pending = 1;
            break;

        default:
            /* Unsupported format.  Confirm with
               CAN_XR_MAC_TX_STATUS_NO_SUCCESS.
               TBD: ts not in scope.  Use 0 for the time being.
            */
            if(mac->primitives.data_conf)
            {
                mac->primitives.data_conf(mac->llc, 0,
                                          identifier, CAN_XR_MAC_TX_STATUS_NO_SUCCESS);
            }
            break;
        }
    }
}

#define CRC_POLYNOMIAL 0x4599 /* It's monic, MSb omitted */

/* From [1], 10.4.2.6.  Update crc considering the LSb of nxtbit.  It
   is meant to be correct, not fast.
*/
static uint16_t crc_nxtbit(uint16_t crc, uint16_t nxtbit)
{
    int crcnxt = ((crc & 0x4000) >> 14) ^ nxtbit;
    crc = (crc << 1) & 0x7FFF; /* Shift in 0 */
    if(crcnxt)  crc ^= CRC_POLYNOMIAL;
    return crc;
}

static void resynchronize_nonce(struct CAN_XR_MAC_State *state)
{
    /* set new nonce */
    uint64_t backup_nonce[2] = {state->src_nonce[0], state->src_nonce[1]};
    memcpy(((uint8_t *) &state->src_nonce[1]) + 3, state->rx_data, 5);
    state->src_nonce[0] = 0;

    /* validate MAC */
    bpmac_pre(state->mac_ctx, (uint8_t *) state->src_nonce, (char *) state->tx_src_mac);

    for(int8_t i = 10; i >= 0; i--) {
        bpmac_update(state->mac_ctx, (state->rx_identifier & (1 << i)), (char *) state->tx_src_mac);
    }
    bpmac_sign(state->mac_ctx, (char *) state->rx_data, 5, (char *) state->tx_src_mac);

    if (!(state->rx_data[5] == state->tx_src_mac[1] && state->rx_data[6] == state->tx_src_mac[2] && state->rx_data[7] == state->tx_src_mac[3]))
    {
        /* if MAC incorrect, reset nonce to old value */
        led_on(led2);
        memcpy(&state->src_nonce, &backup_nonce, 16);
    }
    else
    {
        led_on(led4);
        if (++state->src_nonce[0] == 0)
        {
            state->src_nonce[1]++;
        }
        bpmac_pre(state->mac_ctx, (uint8_t *) state->src_nonce, (char *) state->tx_src_mac);
    }
}

/* Static primitive invoked on all de-stuffed bits after SOF while the
   MAC is receiving.  It performs CRC calculation using crc_nextibt
   and deserialization and recompiling of the frame structure, [1]
   10.3.3.

   TBD:

   - We currently support only CBFF
   - We don't implement OF
   - We don't handle intermission
   - We allow hard synchronization even in the first bit of intermission
*/
static void de_stuffed_data_ind(
    struct CAN_XR_MAC *mac, unsigned long ts, int input_unit)
{
    TRACE(2, "MAC @%lu Common::de_stuffed_data_ind(%d)", ts, input_unit);
    int bit;

    switch(mac->state.rx_fsm_state)
    {
    case CAN_XR_MAC_RX_FSM_IDLE:
        /* SOF received.  Process it, then start receiving the
           identifier.  FSM state transitions related to framing (so,
           not error-related transitions) are taken here and not in
           pcs_data_ind (even though it makes use of the state)
           because they must consider the de-stuffed data stream.
        */
        TRACE(2, "MAC @%lu SOF received (%d)", ts, input_unit);

        /* Disable hard synchronization per [1] 11.3.2.1 c) */
        CAN_XR_PCS_Hard_Sync_Allowed_Req(mac->pcs, 0);
        /* As we do no CRC check, we do not need to initialize and compute it */
        mac->state.field_bits = 10;
        mac->state.rx_identifier = 0;
        mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_IDENTIFIER;
        break;

    case CAN_XR_MAC_RX_FSM_RX_IDENTIFIER:
        /* .field_bits indicates the current bit number within the
           current field, the LSb is zero.  Together with .rx_fsm_state
           it determines where we believe to be within a frame.
        */

        /* Within a field, the MSB shall be transmitted first.  Order
           of bit transmission, [1] 10.8. */
        mac->state.rx_identifier =
            shift_in(mac->state.rx_identifier, input_unit);

        /* in this implementation, all identifier share the same key, so we can start to precompute the MAC here, just
           in case that the identifier indicates an authenticated message. If we have different keys for different
           identifier, it will not work here, but could be precomputed and moved into the bpmac_pre() function.
         */
        bpmac_update(mac->state.mac_ctx, input_unit, (char *) mac->state.tx_src_mac);

        /* Update CRC and switch to the control field if needed. */
        if(mac->state.field_bits-- == 0)
        {
            if (mac->state.rx_identifier > 256)
            {
                mac->state.skip_mac = 1;
                bpmac_reset(mac->state.mac_ctx, (char *) mac->state.tx_src_mac);
            }

            mac->state.field_bits = 1;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_RTR;
        }
        break;

    case CAN_XR_MAC_RX_FSM_RX_RTR:

        /* TBD: RTR bit unchecked, shall be dominant because we do not
           support RTR frames at this time.
        */
        mac->state.rx_rtr = input_unit;
        mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_IDE;
        break;

    case CAN_XR_MAC_RX_FSM_RX_IDE:
        mac->state.rx_ide = input_unit;

        /* TBD: We currently support only CBFF, it must be IDE=0. */
        if(mac->state.rx_ide != 0)
        {
            TRACE(2, "MAC @%lu xEFF formats unsupported", ts);
            mac->state.field_bits = 11;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_ERROR;
        }

        else
        {
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_FDF;
        }

        break;

    case CAN_XR_MAC_RX_FSM_RX_FDF:
        mac->state.rx_fdf = input_unit;

        /* TBD: We currently support only CBFF, it must be FDF=0. */
        if(mac->state.rx_ide != 0)
        {
            TRACE(2, "MAC @%lu FBFF format unsupported", ts);
            mac->state.field_bits = 11;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_ERROR;
        }

        else
        {
            mac->state.field_bits= 3;
            mac->state.rx_dlc = 0;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_DLC;
        }

        break;

    case CAN_XR_MAC_RX_FSM_RX_DLC:

        mac->state.rx_dlc = shift_in(mac->state.rx_dlc, input_unit);
        if(mac->state.field_bits-- == 0)
        {
            /* Calculate how many bits the data field has.  It may be
               empty, skip directly to the CRC in that case, and
               8 byte at a max.
            */
            if (mac->state.rx_dlc > 0)
            {
                /* dlc will transmit the whole length of the payload, we
                * reduce it here by 3 byte, as it will be handled as data MAC.
                */
                if (mac->state.skip_mac)
                {
                    mac->state.field_bits = 8 * ((mac->state.rx_dlc > 8) ? 8: mac->state.rx_dlc) - 1;
                }
                else
                {
                    mac->state.field_bits = 8 * ((mac->state.rx_dlc > 8) ? 5: mac->state.rx_dlc - 3) - 1;
                }
                memset(mac->state.rx_data, 0, sizeof(mac->state.rx_data));
                mac->state.rx_byte = 0;
                mac->state.rx_byte_index = 0;
                mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_DATA;
            }
            else
            {
                mac->state.field_bits = 14;
                mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_CRC;
            }
        }
	    break;

    case CAN_XR_MAC_RX_FSM_RX_DATA:

        mac->state.rx_byte = shift_in(mac->state.rx_byte, input_unit);

        if (!mac->state.skip_mac) {
            bpmac_update(mac->state.mac_ctx, input_unit, (char *) mac->state.tx_src_mac);
        }

        if(mac->state.field_bits % 8 == 0)
        {
            /* Byte boundary, move reassembled byte from .rx_byte into
               .rx_data[] at the right position.  Even though bits
               within a byte are transmitted big-endian, bytes within
               the data field are transmitted little-endian.  See [1],
               Figures 12-17.Interesting.

               TBD: Are we sure we don't read rx_data[8] in this way?
            */
            mac->state.rx_data[mac->state.rx_byte_index++] = mac->state.rx_byte;

            mac->state.rx_byte = 0;
        }

        if(mac->state.field_bits-- == 0)
        {
            if (mac->state.skip_mac)
            {
                mac->state.field_bits = 14;
                mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_CRC;
            }
            else
            {
                bpmac_finish(mac->state.mac_ctx, (char *) mac->state.tx_src_mac);

                mac->state.mac_byte_index = 1;
                mac->state.tx_mac_shift_reg = shift_prepare(mac->state.tx_src_mac[mac->state.mac_byte_index++], 8);

                CAN_XR_PCS_Set_Fast_Pass(mac->pcs, 1);
                mac->state.field_bits = 23;
                mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_DATA_MAC;
            }
        }
        break;

    case CAN_XR_MAC_RX_FSM_RX_DATA_MAC:
        mac->state.rx_byte = shift_in(mac->state.rx_byte, input_unit);
        CAN_XR_PCS_Data_Req(mac->pcs, input_unit);

        if(mac->state.field_bits == 0)
        {
            CAN_XR_PCS_Reset_Fast_Pass(mac->pcs);
            mac->state.field_bits = 14;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_CRC;
        }

        else if(mac->state.field_bits-- % 8 == 0)
        {
            /* I think this could also be solved more elegant, by using shift_prepare on tx_src_mac[2]
             * and shift it by 16. The tx_mac_shift_reg is 32 bit long and can hold the two byte. This
             * would make this whole else-if block irrelevant and would improve execution time of the
             * FSM. This does also hold for the TX_DATA state of the sender. Could be an improvement
             * for the future. */
            mac->state.tx_mac_shift_reg = shift_prepare(mac->state.tx_src_mac[mac->state.mac_byte_index++], 8);
        }

        break;

    case CAN_XR_MAC_RX_FSM_RX_CRC:
            /* No need to store the CRC being received anywhere, just keep
               going with the CRC calculation.  Due to a well-known
               property, if the received CRC was ok, the calculated CRC
               must be 0 at the end.  Wow, magic! :)
            */
        if(mac->state.field_bits-- == 0)
        {
            /* Assume CRC Ok
             * I do not see any benefit in validating the CRC value at the authenticator. We let other nodes
             * validate the CRC value. The nonce_resync message will be accepted, if other nodes have acknowledged
             * it.
             */
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_CDEL;
        }
        break;

    case CAN_XR_MAC_RX_FSM_RX_CDEL:

        if(input_unit != 1)
        {
            TRACE(9, ">>> MAC @%lu CDEL form error", ts);
            mac->state.field_bits = 11;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_ERROR;
            led_on(led2);
        }

        else
        {
            /* Acknowledge the frame by starting the transmission of a
               dominant ACK bit at the next bit boundary.  PCS takes
               care of bit boundary synchronization.

               TBD: If we are transmitting, we should not acknowledge
               our own frame.  At this time we data data_req_pending,
               but we don't have data_req_active, we probably need it.

               TBD: This is also a (strong) hint at how to implement
               SRR mode.
            */
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_ACK;
        }
        break;

    case CAN_XR_MAC_RX_FSM_RX_ACK:

        if(input_unit != 0)
        {
            mac->state.field_bits = 11;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_ERROR;
            led_on(led2);
        }

        else
        {
            /* Stop transmitting the dominant ACK bit */
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_ADEL;
        }

        break;

    case CAN_XR_MAC_RX_FSM_RX_ADEL:

        if(input_unit != 1)
        {
            mac->state.field_bits = 11;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_ERROR;
            led_on(led2);
        }

        else
        {
            mac->state.field_bits = 2;  // reduced EOF length, to buy some time
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_RX_EOF;
        }
        break;

    case CAN_XR_MAC_RX_FSM_RX_EOF:
        /* The EOF field consists of 7 recessive bits ([1] 10.4.2.8).
           However, "the value of the last bit of EOF shall not
           inhibit frame validation and a dominant value shall not
           lead to a form error.  A receiver that detects a dominant
           bit at the last bit of EOF shall respond with an OF" ([1]
           10.7).

           TBD: We don't implement OF, so we simply ignore the 7th bit
           of EOF for the time being.
        */
        if(input_unit != 1 && mac->state.field_bits != 0)
        {
            mac->state.field_bits = 11;
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_ERROR;
            led_on(led2);
        }

        else if(mac->state.field_bits-- == 0)
        {
            if (mac->state.skip_mac) {
                if (mac->state.rx_identifier == 385)
                {
                    /* nonce_resync message */
                    led_on(led3);
                    resynchronize_nonce(&mac->state);
                }
            }
            else
            {
                reset_leds();
                if (++mac->state.src_nonce[0] == 0)
                {
                    mac->state.src_nonce[1]++;
                }
                bpmac_pre(mac->state.mac_ctx, (uint8_t *) mac->state.src_nonce, (char *) mac->state.tx_src_mac);
            }
            /* TBD: We don't handle intermission properly.  Moreover,
               we shouldn't allow hard synchronization in the first
               bit of intermission 11.3.2.1 c)
            */
            CAN_XR_PCS_Hard_Sync_Allowed_Req(mac->pcs, 1);
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_IDLE;
        }
        break;

    default:
        break;
    }
}

/* PCS_Data.Indicate primitive invoked by PCS the arrival of a bit.
   This is the starting point for MAC-layer processing.

   Bus off condition unchecked / recovery unsupported.
   FD tolerant / FD enabled MAC unsupported.
*/
static void pcs_data_ind(
    struct CAN_XR_MAC *mac, unsigned long ts, int input_unit)
{
    TRACE(2, "MAC @%lu Common::pcs_data_ind(%d)", ts, input_unit);

    /* Handle rx FSM first */
    switch(mac->state.rx_fsm_state)
    {
    case CAN_XR_MAC_RX_FSM_BUS_INTEGRATION:
        TRACE(2, ">>> MAC @%lu bus integration", ts);

        if(input_unit == 0)
        {
            /* Bus dominant @ sample point.  Stay in bus integration
               and reset integration counter. */
            mac->state.bus_integration_counter = 0;
        }

        else
        {
            /* Bus recessive @ sample point.  Keep counting and
               transition to idle after 11 recessive bits.  This way
               of transitioning implies a one-bit delay between
               declaring the idle state and doing anytyhing else in
               the MAC.  TBD: Do we need a bypass?
            */
            if(++mac->state.bus_integration_counter == 11)
            {
                TRACE(2, ">>> MAC @%lu declaring bus idle", ts);

                mac->state.bus_integration_counter = 0;
                mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_IDLE;
            }
        }
        break;

    case CAN_XR_MAC_RX_FSM_IDLE:
        if(input_unit == 0)
        {
            /* SOF received, [1] 10.4.2.2 and 10.4.6.3.  Initialize
               bit de-stuffing state, received 1 bit @ 0. */
            mac->state.nc_bits = 1;
            mac->state.nc_pol = input_unit;
            mac->state.bus_bits = 1;
            mac->state.de_stuffed_bits = 1;
            mac->state.skip_mac = 0;

            de_stuffed_data_ind(mac, ts, input_unit);
        }
        break;

    case CAN_XR_MAC_RX_FSM_RX_IDENTIFIER:
    case CAN_XR_MAC_RX_FSM_RX_RTR:
    case CAN_XR_MAC_RX_FSM_RX_IDE:
    case CAN_XR_MAC_RX_FSM_RX_FDF:
    case CAN_XR_MAC_RX_FSM_RX_DLC:
    case CAN_XR_MAC_RX_FSM_RX_DATA:
    case CAN_XR_MAC_RX_FSM_RX_CRC:
    case CAN_XR_MAC_RX_FSM_RX_CDEL:
        /* Common entry point for all states in which the MAC is
           receiving and bit de-stuffing is needed.  Do it, then call
           de_stuffed_data_ind to continue processing.

           RX_CDEL is in the list because there may be a stuff bit
           after the last bit of CRC, as it must not be considered as
           CDEL by itself.

           TBD: This is probably also a good place to implement bit
           monitoring and detect arbitration loss.  Neither of those
           are implemented for now.
        */
        mac->state.bus_bits++;

        if(mac->state.nc_bits == 5)
        {
            /* Expecting a stuff bit, must be the opposite of nc_pol. */
            if(input_unit == mac->state.nc_pol)
            {
                mac->state.field_bits = 11;
                mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_ERROR;
            }
            else {
                TRACE(2, ">>> MAC @%lu discarding stuff bit @%d", ts, input_unit);
                mac->state.nc_bits = 1;
                mac->state.nc_pol = input_unit;

            }
        }

        else
        {
            if(input_unit != mac->state.nc_pol)
            {
                /* Reset bit counter on polarity change. */
                mac->state.nc_bits = 1;
                mac->state.nc_pol = input_unit;
            }

            else
            {
                /* Same polarity, keep counting */
                mac->state.nc_bits++;
            }

            mac->state.de_stuffed_bits++;
            de_stuffed_data_ind(mac, ts, input_unit);
        }

        break;

    case CAN_XR_MAC_RX_FSM_RX_DATA_MAC:
        /* During data_mac phase, the authentication node needs to wait if the
         * sender transmits a stuff bit. Additionally, instead of the received
         * signal, the modified one will count for bit stuffing.
        */
        mac->state.bus_bits++;
        if(mac->state.nc_bits == 5)
        {
            /* Expecting a stuff bit, must be the opposite of nc_pol. */
            if(input_unit == mac->state.nc_pol)
            {
                /* Current bit is same polarity as last 5 bits -> Stuff Error */
                CAN_XR_PCS_Reset_Fast_Pass(mac->pcs);
                mac->state.field_bits = 11;
                mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_ERROR;
                led_on(led2);
            }
            else
            {
                /* Read stuff bit. Drop it and continue */
                CAN_XR_PCS_Data_Req(mac->pcs, input_unit);  /* set value to stuff bit value */
                mac->state.nc_bits = 1;
                mac->state.nc_pol = input_unit; /* set current value to stuff bit value */

            }
        }
        else
        {
            uint8_t bit;
            shift_out(bit, mac->state.tx_mac_shift_reg);
            bit = bit ^ input_unit;
            /* Overwritten signal will be an edge and change bit polarity -> Reset counter */
            if( bit != mac->state.nc_pol)
            {
                /* Reset bit counter. */
                mac->state.nc_bits = 1;
                /* Polarity will be overwritten signal */
                mac->state.nc_pol = bit;
            }
            /* Overwriting signal will not cause an edge */
            else
            {
                /* Same polarity, keep counting */
                mac->state.nc_bits++;
            }

            mac->state.de_stuffed_bits++;
            de_stuffed_data_ind(mac, ts, bit);
        }

        break;
    case CAN_XR_MAC_RX_FSM_RX_ACK:
        /* TBD: This is probably a good place to detect ACK errors.
           The transmitter transmits a recessive bit, we should sample
           dominant here.
        */
    case CAN_XR_MAC_RX_FSM_RX_ADEL:
    case CAN_XR_MAC_RX_FSM_RX_EOF:
        /* Bypass bit de-stuffing in the frame trailer [1] 10.5 last
           sentence.  See above for the special tratment of the
           CAN_XR_MAC_RX_FSM_RX_CDEL state.
        */
        de_stuffed_data_ind(mac, ts, input_unit);
        break;
    case CAN_XR_MAC_RX_FSM_ERROR:
            /* TBD: Very simple error recovery, transmit recessive at next
                bit boundary, enable hard synchronization, reset bpmac computation
                bring the tx automaton to idle and the rx automaton to the bus
                integration state.
            */
        CAN_XR_PCS_Data_Req_Stop(mac->pcs);
        if (mac->state.field_bits-- == 0)
        {
            bpmac_reset(mac->state.mac_ctx, (char *) mac->state.tx_src_mac);
            CAN_XR_PCS_Hard_Sync_Allowed_Req(mac->pcs, 1);
            mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_IDLE;
        }
        break;
    default:
        CAN_XR_PCS_Data_Req_Stop(mac->pcs);
        CAN_XR_PCS_Hard_Sync_Allowed_Req(mac->pcs, 1);
        mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_BUS_INTEGRATION;
        mac->state.tx_fsm_state = CAN_XR_MAC_TX_FSM_IDLE;
        break;
    }
}

void CAN_XR_MAC_Common_Init(
    struct CAN_XR_MAC *mac,
    struct CAN_XR_PCS *pcs)
{
    TRACE(2, "CAN_XR_MAC_Common_Init");

    /* No LLC for now, link to PCS */
    mac->llc = NULL;
    mac->pcs = pcs;

    /* MAC state initialization (implementation-independent part.
       The MAC FSM starts in bus integration state, [1] 10.9.4.

    */
    mac->state.rx_fsm_state = CAN_XR_MAC_RX_FSM_BUS_INTEGRATION;
    mac->state.bus_integration_counter = 0;

    mac->state.tx_fsm_state = CAN_XR_MAC_TX_FSM_IDLE;
    mac->state.data_req_pending = 0;

    mac->state.skip_mac = 0;

    /* load keys and nonce into DATA_MAC_Storage
     * Additionally set state pointer, which would be
     * done depending on frame ID */
    uint8_t src_key[] = {0x17, 0x07, 0x17, 0x07, 0x17, 0x07, 0x17, 0x07, 0x17, 0x07, 0x17, 0x07, 0x17, 0x07, 0x17, 0x07};
    uint8_t src_key_nonce[] = {0x22, 0x11, 0x27, 0x09, 0x22, 0x11, 0x27, 0x09,0x22, 0x11, 0x27, 0x09 ,0x22, 0x11, 0x27, 0x09};

    memcpy(mac->storage.src_mac_key, src_key, 16);
    mac->state.src_key = mac->storage.src_mac_key;

    memcpy(mac->storage.src_nonce_key, src_key_nonce, 16);
    mac->state.src_nonce_key = mac->storage.src_nonce_key;

    memset(mac->storage.res_nonce, 0, 16);
    memset(mac->storage.src_nonce, 0, 16);

    memset(mac->state.tx_src_mac, 0, 16);
    mac->state.mac_ctx = &(mac->storage.ctx);

    bpmac_init((char *) mac->state.src_key, (char *) mac->state.src_nonce_key, 8, mac->state.mac_ctx);
    bpmac_pre(mac->state.mac_ctx, (uint8_t *) mac->state.src_nonce, (char *) mac->state.tx_src_mac);

    /* No data_ind, data_conf for now.  Link the common, static
       data_req, may be overridden by implementation-specific
       initialization function at a later time.
    */
    mac->primitives.data_ind = NULL;
    mac->primitives.data_conf = NULL;
    mac->primitives.data_req = mac_data_req;
    mac->primitives.ext_tx_data_ind = NULL;

    /* Link PCS to MAC, register the common, static data_ind */
    CAN_XR_PCS_Set_MAC(pcs, mac);
    CAN_XR_PCS_Set_Data_Ind(pcs, pcs_data_ind);
}

void CAN_XR_MAC_Set_LLC(struct CAN_XR_MAC *mac, struct CAN_XR_LLC *llc)
{
    mac->llc = llc;
}

void CAN_XR_MAC_Set_Data_Ind(
    struct CAN_XR_MAC *mac, CAN_XR_MAC_Data_Ind_t data_ind)
{
    mac->primitives.data_ind = data_ind;
}

void CAN_XR_MAC_Set_Data_Conf(
    struct CAN_XR_MAC *mac, CAN_XR_MAC_Data_Conf_t data_conf)
{
    mac->primitives.data_conf = data_conf;
}

void CAN_XR_MAC_Set_Ext_Tx_Data_Ind(
    struct CAN_XR_MAC *mac, CAN_XR_MAC_Ext_Tx_Data_Ind_t ext_tx_data_ind)
{
    mac->primitives.ext_tx_data_ind = ext_tx_data_ind;
}


void CAN_XR_MAC_Data_Req(
    struct CAN_XR_MAC *mac,
    uint32_t identifier, enum CAN_XR_Format format, int dlc, uint8_t *data)
{
    if(mac->primitives.data_req)
    {
        mac->primitives.data_req(mac, identifier, format, dlc, data);
    }
}
