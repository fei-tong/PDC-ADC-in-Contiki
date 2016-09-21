# PDC-ADC
Pipelined Data Collection (PDC) takes into account both the pipelined data collection and the underlying schedule synchronization over duty-cycled radios practically and comprehensively. It integrates all its components in a natural and seamless way to simplify the protocol implementation and to achieve a high energy efficiency and low packet delivery latency. Based on PDC, an Adaptive Data Collection (ADC) protocol is further proposed to achieve dynamic duty-cycling and free addressing, which can improve network heterogeneity, load adaptivity, and energy efficiency. Both PDC and ADC have been implemented in a pioneer open-source operating system for the Internet of Things, and evaluated through a testbed built based on two hardware platforms, as well as through emulations.

## PDC 
PDC has as been published in IEEE Sensors Journal

F. Tong, R. Zhang, J. Pan, “One Handshake Can Achieve More: An Energy-Efficient, Practical Pipelined Data Collection for Duty-Cycled Sensor Networks”, IEEE Sensors Journal, 16(9):3308–3322, May 2016.

http://ieeexplore.ieee.org/document/7397877/?arnumber=7397877

## ADC
ADC has been accepted by Qshine 2016:

F. Tong, J. Pan, “Adaptive Data Collection with Free Addressing and Dynamic Duty-Cycling for Sensor Networks”, Accepted for publication, EAI International Conference on Heterogeneous Networking for Quality, Reliability, Security and Robustness (QShine), May 2016.

## How to use the code
The code was developed based on contiki 3.0. The provided code has been tested and worked fine. To run the code, 
* put the folder `PDC_ADC` in `contiki/examples`
* run cooja simulator
* open `PDC_ADC/PDC_board_sim/ADC-PDC-7.csc` in cooja simulator

## Parameters Configuration
The code parameters can be configured in `PDC_ADC/PDC_board_sim/Makefile`. For example, if `ADCSC_SUPPORT=0`, only it is just PDC without the ADC function; othewise ADC is supported. The comments for other parameters can be found in the `Makefile`.
