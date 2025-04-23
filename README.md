# Compact And Instantaneous Bus Authentication (CAIBA)
This repository provides the implementation of the security protocol proposed in the the paper "CAIBA: Multicast Source Authentication for CAN Through Reactive Bit Flipping", accepted for publication at EuroS&P 2025.
It provides source authentication capabilities for multicast communication, using fast and easy to compute Message Authentication Codes (MACs).

### Getting Started
The implementation is build on the code of the [Software-Defined CAN Controller (SDCC)](https://github.com/minimap-xl/sdcc), proposed by Cena et al. [1].
Each directory contains the implementation of one type of node, i.e., authenticator, sender and receiver, with the implementation of the receiving nodes being mostly unchanged from the original SDCC.
The code structure for each node is taken from SDCC:

- `/src/CAN_XR_Controller/`: Contains the platform-independent implementation of the respective node type.
- `/src/Cross/`: Contains the platform dependent implementation of the Physical Medium Attachment, which also covers the register configuration of the used microcontroller.
- `/src/Cross_Programs/`: Contains the main program of the respective node type, i.e., initialization, transmission calls and handling of received messages at sender and receiver.

### Prerequisites
The platform dependent code contains the register configuration for the [mbed LPC1768](https://os.mbed.com/platforms/mbed-LPC1768/) development and prototyping board, which uses the microcontroller suggested by the authors of SDCC.
Using other microcontrollers requires the modification of the register configuration in the `/src/Cross/CAN_XR_PMA_GPIO.c` file in each directory. 
For the connection to the bus, we used four [TI SN65HVD230](https://www.ti.com/product/de-de/SN65HVD230), one for each node and an additional one at the authenticator.
Each directory contains information about the connection of the microcontrollers to the transceivers.
All transceivers have to be connected to a properly-terminated CAN bus, with one at the authenticator being connected inverted. 

For an easy cross-platform build of each node, we used [platformio](https://platformio.org/).

### Building and Running
Each node must be built independently.
For using the platformio CLI, entering the respective directory and run
```bash
pio run
```
The final binary can be found in each directory at `/.pio/build/lpc1768/firmware.bin` (path might vary, depending on configured microcontroller) and can be uploaded onto the microcontroller.

---
[1] Gianluca Cena, Ivan Cibrario Bertolotti, Tingting Hu, Adriano Valenzano. "On a software-defined CAN controller for embedded systems." Computer Standards & Interfaces, 63, 43-51. 2019 [https://doi.org/10.1016/j.csi.2018.11.007](https://doi.org/10.1016/j.csi.2018.11.007)
