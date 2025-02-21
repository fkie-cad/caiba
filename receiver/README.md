# Receiver

Implementation of a receiving node for CAIBA.
The code is almost unchanged from the original code of the [Software-Defined CAN Controller (SDCC)](https://github.com/minimap-xl/sdcc).

### Hardware Setup
For setup, connect one CAN transceiver to the microcontroller.
In the current configuration, we use the following connection of transceiver and GPIO pins ([pin reference for mbed LPC1768 board](https://os.mbed.com/users/fraserphillips/notebook/mbed-gpio-pin-table/)):

| GPIO port | mbed pin | CAN transceiver pin |
|-----------|----------|---------------------|
| 0.5       | PIN29    | Tx                  |
| 0.4       | PIN30    | Rx                  |

For a different pin selection or a different microcontroller, change the register configuration in the [CAN_XR_PMA_GPIO.c](./src/Cross/CAN_XR_PMA_GPIO.c) file.
