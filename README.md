# PDC-ADC
Pipelined Data Collection (PDC) takes into account both the pipelined data collection and the underlying schedule synchronization over duty-cycled radios practically and comprehensively. It integrates all its components in a natural and seamless way to simplify the protocol implementation and to achieve a high energy efficiency and low packet delivery latency. Based on PDC, an Adaptive Data Collection (ADC) protocol is further proposed to achieve dynamic duty-cycling and free addressing, which can improve network heterogeneity, load adaptivity, and energy efficiency. Both PDC and ADC have been implemented in Contiki 3.0, and evaluated through a testbed built based on two hardware platforms, as well as through Cooja simulation.

## Data Collection in duty-cycled LSN
The work has been published in IEEE Internet of Things Journal.

F. Tong, S. He, J. Pan, “Modeling and Analysis for Data Collection in Duty-Cycled Linear Sensor Networks with Pipelined-Forwarding Feature”, IEEE Internet of Things Journal, 6(6):9489-9502, Dec. 2019.

https://ieeexplore.ieee.org/document/8765632

## PDC 
The work on PDC has as been published in IEEE Sensors Journal

F. Tong, R. Zhang, J. Pan, “One Handshake Can Achieve More: An Energy-Efficient, Practical Pipelined Data Collection for Duty-Cycled Sensor Networks”, IEEE Sensors Journal, 16(9):3308–3322, May 2016.

http://ieeexplore.ieee.org/document/7397877/?arnumber=7397877

## ADC
The work on ADC has been accepted by Qshine 2016:

F. Tong, J. Pan, “Adaptive Data Collection with Free Addressing and Dynamic Duty-Cycling for Sensor Networks”, Accepted for publication, EAI International Conference on Heterogeneous Networking for Quality, Reliability, Security and Robustness (QShine), May 2016.

Its journal version is recently published in MONET:

F. Tong, J. Pan, “ADC: an Adaptive Data Collection Protocol with Free Addressing and Dynamic Duty-Cycling for Sensor Networks”, Mobile Networks and Applications 2017. (https://link.springer.com/article/10.1007%2Fs11036-017-0850-9)

## How to use the code
The code was developed based on contiki 3.0. The provided code has been tested and worked fine. To run the code, 
* Copy the folder `PDC_ADC` in `contiki/examples`
* Run cooja simulator
* Load `PDC_ADC/PDC_board_sim/ADC-PDC-7.csc` in cooja simulator

## Parameters configuration
The code parameters can be configured in `PDC_ADC/PDC_board_sim/Makefile`. For example, if `ADCSC_SUPPORT=0`, it is just PDC without the ADC function; othewise ADC is supported. The comments for other parameters can be found in the `Makefile`.
