
<img src=images/iotera_logo.png width="10%" height="10%"></br>
# Bluetera Firmware
Firmware for the BLE-enabled Bluetera module.</br>

<img src=images/bluetera_module_front.png width="20%" height="20%" hspace="20"/> 
<img src=images/bluetera_module_back.png width="20%" height="20%" hspace="20"/>

Visit our website: https://ioteratech.com
## Getting Started

Bluetera is an open source IoT platform for the development of smart and connected products. The platform includes:
* Bluetera Hardware module - repository [here]()
* Bluetera Firmware (this repository)
* Blutera SDK - repository [here]()

This guide provides a detailed description on how to build and debug the Bluetera firmware. If you just want to run the sample code without modifying the firmware - checkout the complementary [SDK repository]().


## Building the project
### Prerequisites
* Windows 7 and later (should work, but not tested, on Linux nor Mac).
* Nordic Semiconductor's nRF5 SDK, version 15.2.0 - download [here](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download#infotabs)
* GNU Tools ARM Embedded<sup id="a1">[1](#f1)</sup>, version '6 2017-q2-update' or higher - download [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).
  
if might also want to install:
* Visual Studio Code IDE - download [here](https://code.visualstudio.com)
* Invensense Embedded Motion Driver<sup id="a2">[2](#f1)</sup> (only if you intend to modify the Bluetera IMU driver) - download [here](https://www.invensense.com)

### Installing
You can more-or-less follow the instructions of [this blog](https://devzone.nordicsemi.com/tutorials/b/getting-started/posts/development-with-gcc-and-eclipse), skipping Eclipse-related stuff (that is, unless you want to use Eclipse as your IDE :-)):

* Unzip nRF5-SDK to some folder, preferebly with not spaces in the path name. 
* Install the ARM compiler
* Go to the SDK folder, and make sure the build script *<SDK_ROOT>\components\toolchain\gcc\Makefile.windows* points to the correct version of the ARM compiler. It should look something like:
```
GNU_INSTALL_ROOT := C:/Program Files (x86)/GNU Tools ARM Embedded/6 2017-q2-update/bin/
GNU_VERSION := 6.3.1
GNU_PREFIX := arm-none-eabi
```

#### Test your installation
If all went well, you should be able to build Nordic's sample project:
* Open a command-line under *<SDK_ROOT>\examples\peripheral\blinky\<BOARD>\s132\armgcc* <br/>(*BOARD* is *pca10040* for nRF52832-DK, and *pca10056* for nRF52840-DK).
* Run *make*
* Check if the firmware image file was created - *_build\nrf52832_xxaa.hex*

### Build
* Set an environment variable *SDK_ROOT* to the nRF-SDK top folder (e.g. *C:\dev\nordic\sdk\nRF5_SDK_15.2.0_9412b96*).
* Clone this repository, and open a command-line in the top folder
* Run *make*
* The generated firmware image is *_build\bluetera.hex*

## Programming / Debugging the Device
### Prerequisite
* Bluetera module (either with or without battery)
* Nordic Semiconductor's Developement Kits ([nRF52832-DK](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/nRF52-DK) / [nRF52840-DK](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/nRF52840-DK)). 
* A Windows 10 machine, with the following software: 
  * Segger J-Link software for Nordic - download [here](https://www.segger.com/downloads/jlink#J-LinkSoftwareAndDocumentationPack) 
  * nRF command line tools - download [here](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF5-Command-Line-Tools/Download#infotabs)
### Preperation
  * Solder 4 wires to Bluetera SWD pads - VCC, GND, CLK and SIO, shown on the right side of the following image: 
  
  <img style="display: block; margin-left:auto;  margin-right:auto;"  src=images/bluetera_module_front_signals.png width="60%" height="60%"/> 

* Prepare the PCA10040 / PCA10056 board: 
  *  Make sure the main power switch is off
  *  In PCA10056, SW9 (nRF power source) should be on 'VDD', SW6 to 'DEFAULT'
  *  Connect the GND_DETECT pin of header P20 to GND pin of header P1
*  Connect the SWD wires to the board:
   *  Bluetera GND to P1.GND pin
   *  Bluetera VCC to P20.VTG pin
   *  Bluetera CLK to P20.SWD_CLK pin
   *  Bluetera SIO to P20.SWD_IO pin
* If the Bluetera is without battery, you can also power it from the board, by shorting P20.VTG to P20.VDD_nRF pin.

The Following images illustrate the connection without a battery: 
     
![](images/bluetera_PRG_4_w300.jpg)

### Programming:
  * Connect the Development Board to a PC via USB.
  * Turn it on, and wait until drivers are installed.
  * If all works well, a new virtual drive named 'JLINK' will be added to your machine.
  * Drag-and-Drop the firmware image file: *_build\bluetera.hex* to the JLink virtual drive, and wait for the programming to complete.
      
## First Usage (Windows Logger Demo)
Iotera logger demo is a Windows open source app that illustrates the usage of the IMU module and data logging. The source code and app can be found in [this repository]()
1. Prerequisite:
 * Machine with windows operating system 
 * Bluetera device with firmware that fits the Logger APIs
 * BLE CSR Dongle and driver 
 * Logger demo software 
2. Usage:
* Run ../ioteraDemo.exe
* The followinbg window should open 

![Blutera Logger](images/logger-1.jpg)

* Click Logging and enable. You can also set the log path 

![Blutera Logger](images/logger-2.jpg)

* Click start (1). The app should pair to the first BT module it detects (2) and outline its MAC number 

![Blutera Logger](images/logger-3.jpg)

* You can stop the logging and also reset the cube by clicking the corresponding button at menu (1)
* The data section (3) presents Acceleration data (3 axis); Quaternion data and Euler angles - all in realtime (default data sampling is 50hz)
* The graphical section (4) presents a rotating cube (based on quaternion)
* The logger records the data into a CSV file which you can export and analyze
* 
## Authors
* **Tomer Abramovich** - [Tensor Technologies]()
* **Boaz Aizenshtark** - [Tensor Technologies]()
* **Avi Rabinovich** - [Tensor Technologies]()

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Notes
<sup id="f1">1</sup> Nordic support other IDEs and toolchains. This repository was only built and tested using GCC. [↩](#a1)

<sup id="f2">2</sup> InvenSense driver is not open-source. You can, however, request the sources from TDK. Bluetera firmware includes the Invensense driver as a pre-compiled library [↩](#a1)