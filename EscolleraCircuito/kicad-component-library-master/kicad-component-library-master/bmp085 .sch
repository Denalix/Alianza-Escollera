EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:pin
LIBS:isp_avr
LIBS:sop8l
LIBS:MAX485
LIBS:mpu6050
LIBS:bmp085
LIBS:hmc5883
LIBS:ds1307
LIBS:mylib
LIBS:kk-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 5
Title ""
Date "20 jun 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L BMP085 b1
U 1 1 53A48A3A
P 5900 2950
F 0 "b1" H 5900 2200 60  0000 C CNN
F 1 "BMP085" H 5900 2950 60  0000 C CNN
F 2 "~" H 5900 2950 60  0000 C CNN
F 3 "~" H 5900 2950 60  0000 C CNN
	1    5900 2950
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 53A48B0B
P 5050 3750
F 0 "C6" H 5050 3850 40  0000 L CNN
F 1 "0.1uf" H 5056 3665 40  0000 L CNN
F 2 "~" H 5088 3600 30  0000 C CNN
F 3 "~" H 5050 3750 60  0000 C CNN
	1    5050 3750
	1    0    0    1   
$EndComp
$Comp
L C C5
U 1 1 53A48B24
P 4500 3450
F 0 "C5" H 4500 3550 40  0000 L CNN
F 1 "0.1if" H 4506 3365 40  0000 L CNN
F 2 "~" H 4538 3300 30  0000 C CNN
F 3 "~" H 4500 3450 60  0000 C CNN
	1    4500 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3550 5300 3550
Wire Wire Line
	4500 3250 4850 3250
Wire Wire Line
	4850 3250 4850 3400
Wire Wire Line
	4850 3400 5300 3400
Wire Wire Line
	4500 3650 4500 3950
Wire Wire Line
	4500 3950 5050 3950
$Comp
L R R3
U 1 1 53A48B44
P 7050 2800
F 0 "R3" V 7130 2800 40  0000 C CNN
F 1 "10k" V 7057 2801 40  0000 C CNN
F 2 "~" V 6980 2800 30  0000 C CNN
F 3 "~" H 7050 2800 30  0000 C CNN
	1    7050 2800
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 53A48B53
P 7350 2800
F 0 "R4" V 7430 2800 40  0000 C CNN
F 1 "10k" V 7357 2801 40  0000 C CNN
F 2 "~" V 7280 2800 30  0000 C CNN
F 3 "~" H 7350 2800 30  0000 C CNN
	1    7350 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3050 7000 3050
Wire Wire Line
	7000 3050 7000 3250
Wire Wire Line
	7000 3250 6500 3250
Wire Wire Line
	6500 3400 7350 3400
Wire Wire Line
	7350 3400 7350 3050
Wire Wire Line
	7050 2550 7350 2550
Text Label 7200 2550 0    60   ~ 0
vcc
Text HLabel 9050 2600 0    60   Input ~ 0
vcc
Text HLabel 9050 2900 0    60   Input ~ 0
gnd
Text HLabel 7950 2800 0    60   Input ~ 0
sda
Text HLabel 7950 3000 0    60   Input ~ 0
scl
Text Label 9050 2600 0    60   ~ 0
vcc
Text Label 9050 2900 0    60   ~ 0
gnd
Text Label 5300 3100 0    60   ~ 0
gnd
Text Label 4500 3250 0    60   ~ 0
vcc
Text Label 5050 3550 0    60   ~ 0
vcc
Text HLabel 5150 3200 0    60   Input ~ 0
eoc
Wire Wire Line
	5150 3200 5150 3250
Wire Wire Line
	5150 3250 5300 3250
Text HLabel 6650 2900 0    60   Input ~ 0
xclr
Wire Wire Line
	6500 3100 6650 3100
Wire Wire Line
	6650 3100 6650 2900
$EndSCHEMATC
