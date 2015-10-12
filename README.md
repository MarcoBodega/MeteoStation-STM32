
This project implements a simple station for monitoring temperature, humidity and gas. 
It just provide its output via serial consol UART interface (9600baud, no hw flow).

Temperature and Humidity are read in PWM Input mode from the DH11 sensor, while Methan level is read from MQ-2 sensor which provides an analogue outputs.

This branch implements this project on the STM32VL discovery board from St-Microelectronics.
