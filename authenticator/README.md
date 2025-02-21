# Authenticator

Implementation of the authenticater node of CAIBA.
The node computes the source authentication tag in parallel to the transmission of the payload and XORs it to the following tag in the message to remove it during the transmission.

### Hardware Setup
For setup, connect **two** CAN transceiver to the microcontroller, with one being connected inverted, i.e., `CAN high` of the transceiver to `CAN low` of the CAN bus and vice versa.
In the current configuration, we use the following connection of transceiver and GPIO pins ([pin reference of mbed LPC1768 board](https://os.mbed.com/users/fraserphillips/notebook/mbed-gpio-pin-table/)):

| GPIO port | mbed pin | CAN transceiver pin | CAN transceiver bus connection |
|-----------|----------|---------------------|--------------------------------|
| 0.5       | PIN29    | Tx                  | normal                         |
| 0.4       | PIN30    | Rx                  | normal                         |
| 0.10      | PIN28    | Tx                  | inverted                       |

For a different pin selection or a different microcontroller, change the register configuration in the [CAN_XR_PMA_GPIO.c](./src/Cross/CAN_XR_PMA_GPIO.c) file.




