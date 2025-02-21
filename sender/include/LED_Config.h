//
// Created by Freddy - 09.03.2023.
//

#ifndef SDCC_LED_CONFIG_H
#define SDCC_LED_CONFIG_H

/*
 * Register definition of the 4 LEDs
 */
#define ADDR_REG32 (volatile uint32_t *)
#define REG32(x)   (* (ADDR_REG32 (x)))

#define FIO1_DIR2  REG32(0x2009C022) /* (UM 10360 p.124) */
#define FIO1_PIN  REG32(0x2009C034)
#define FIO1_SET  (0x2009C038)
#define FIO1_CLR  (0x2009C03C)

#define led1        18  // pin18
#define led2        20  // pin20
#define led3        21  // pin21
#define led4        23  // pin23
static volatile unsigned *fio1set   = (unsigned *)FIO1_SET;
static volatile unsigned *fio1clr   = (unsigned *)FIO1_CLR;

#define enable_leds()   FIO1_DIR2 |= 0b10110100;
#define led_on(x)   *fio1set = (1 << x)
#define led_off(x)  *fio1clr = (1 << x)
#define reset_leds() *fio1clr = (0b101101 << 18)
#define led_set_all() *fio1set = (0b101101 << 18)
//((FIO0PIN & PORT_0_4_MASK) ? 1 : 0)
#define led_status(x)    ((FIO1_PIN & (1 << x)) ? 1 : 0)
#define invert_led(x)           \
    do {                        \
         if (led_status(x))     \
        {                       \
            led_off(x);         \
        }                       \
        else                    \
        {                       \
            led_on(x);          \
        };                      \
    } while (0)                 \

/*
* Definitions for DEBUG PINS
*/
#define FIO1_DIR3 REG32(0x2009C023)
#define enable_debug_pins() FIO1_DIR3 |= 0b11000000;
#define debug_pin   31
#define debug_pin2  30
#define dbug_on()     *fio1set = (1 << debug_pin);
#define dbug2_on()    *fio1set = (1 << debug_pin2)
#define dbug_off()    *fio1clr = (1 << debug_pin)
#define dbug2_off()    *fio1clr = (1 << debug_pin2)


#endif //SDCC_LED_CONFIG_H
