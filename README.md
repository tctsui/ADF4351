# ADF4351 Wideband RF Synthesizer

## Introduction
This porject is designed to incorporate Analog Devices ADF 4351 chip with Teensy 3.6. On the bottom right corner, there are two slots reserved for two different TCXO oscillators 535L250X2GT5 and XNCLH25M000THJA0P0. Only one of them should be used. The ADF4351 can be controlled using SPI by the Teensy 3.6 on-board or externally using the header pin when the Teensy is unplugged. The Teensy 3.6 is also powered by this board, so please cut the 5V pad according to https://www.pjrc.com/teensy/external_power.html  before use. The library used to control the ADF4351 chip is mostly based on the code written by David Fannin (dfannin). The register writing part of this code is adapted from the code written by Neal Pisenti (npisenti). The library is still under development, so use it with cautions.

## Hardware
### TXCO oscillators
This PCB is designed such that either 535L250X2GT5 or XNCLH25M000THJA0P0 can be used. If oscillator with frequency other than 25MHz, a corresponding modifcation to the SPI control library has to be made.
### Fast Lock Loop Filter Topology
In order to increase the lock speed, the board is modified according to page 23 in the ADF4351 manual.(a 0 Ohm on R% and replace r4 with a 1k Ohm resistor). The SPI control library in this repository is also adapted to work under the fast lock.

### Teensy 3.6
The Teensy 3.6 is designed to be directly power by the board, so please cut the 5V pad according to https://www.pjrc.com/teensy/external_power.html  before use.

## Software
Most of code in the .cpp is based on the code written by David Fannin (dfannin). The register writing part of the code is adapted from the code written by Neal Pisenti (npisenti). 
I modidified the code to enable the fast lock functionality and calculate the different parameters required for the fast lock operation. Moreover, I added a new function FastScanSetF which only updates one register when call instead of 6. Before using FastScanSetF, FastscanabilityCheckhas to be called to set the remaining 5 registers.


## References

+ [ADF4351 Product Page](https://goo.gl/tkMjw6) Analog Devices
+ [SV1AFN ADF4351 Board](https://www.sv1afn.com/adf4351m.html) by Makis Katsouris, SV1AFN
+ [Big Number Arduino Library](https://github.com/nickgammon/BigNumber) by Nick Gammon

## Picture
<img src="./ADF4351.jpg" height ="800">
