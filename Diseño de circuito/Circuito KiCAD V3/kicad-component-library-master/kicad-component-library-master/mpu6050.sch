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
Sheet 4 5
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
L MPU6050 U1
U 1 1 53A471A4
P 5200 3100
AR Path="/53A47160/53A471A4" Ref="U1"  Part="1" 
AR Path="/53A4A568/53A49A26/53A471A4" Ref="U1"  Part="1" 
F 0 "U1" H 5250 1300 60  0000 C CNN
F 1 "MPU6050" H 5250 3100 60  0000 C CNN
F 2 "~" H 5200 3050 60  0000 C CNN
F 3 "~" H 5200 3050 60  0000 C CNN
	1    5200 3100
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 53A472F5
P 5500 2800
AR Path="/53A47160/53A472F5" Ref="C3"  Part="1" 
AR Path="/53A4A568/53A49A26/53A472F5" Ref="C3"  Part="1" 
F 0 "C3" H 5500 2900 40  0000 L CNN
F 1 "2.2nf" H 5506 2715 40  0000 L CNN
F 2 "~" H 5538 2650 30  0000 C CNN
F 3 "~" H 5500 2800 60  0000 C CNN
	1    5500 2800
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 53A47304
P 6500 4550
AR Path="/53A47160/53A47304" Ref="C4"  Part="1" 
AR Path="/53A4A568/53A49A26/53A47304" Ref="C4"  Part="1" 
F 0 "C4" H 6500 4650 40  0000 L CNN
F 1 "0.1uf" H 6506 4465 40  0000 L CNN
F 2 "~" H 6538 4400 30  0000 C CNN
F 3 "~" H 6500 4550 60  0000 C CNN
	1    6500 4550
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 53A47313
P 5350 5100
AR Path="/53A47160/53A47313" Ref="C2"  Part="1" 
AR Path="/53A4A568/53A49A26/53A47313" Ref="C2"  Part="1" 
F 0 "C2" H 5350 5200 40  0000 L CNN
F 1 "0.1uf" H 5356 5015 40  0000 L CNN
F 2 "~" H 5388 4950 30  0000 C CNN
F 3 "~" H 5350 5100 60  0000 C CNN
	1    5350 5100
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 53A47322
P 5050 5150
AR Path="/53A47160/53A47322" Ref="C1"  Part="1" 
AR Path="/53A4A568/53A49A26/53A47322" Ref="C1"  Part="1" 
F 0 "C1" H 5050 5250 40  0000 L CNN
F 1 "10NF" H 5056 5065 40  0000 L CNN
F 2 "~" H 5088 5000 30  0000 C CNN
F 3 "~" H 5050 5150 60  0000 C CNN
	1    5050 5150
	1    0    0    -1  
$EndComp
Text HLabel 7500 3450 0    60   Input ~ 0
vcc
Text HLabel 7500 3300 0    60   Input ~ 0
gnd
$Comp
L R R1
U 1 1 53A47B10
P 4750 2700
AR Path="/53A47160/53A47B10" Ref="R1"  Part="1" 
AR Path="/53A4A568/53A49A26/53A47B10" Ref="R1"  Part="1" 
F 0 "R1" V 4830 2700 40  0000 C CNN
F 1 "R" V 4757 2701 40  0000 C CNN
F 2 "~" V 4680 2700 30  0000 C CNN
F 3 "~" H 4750 2700 30  0000 C CNN
	1    4750 2700
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 53A47B1F
P 4750 2900
AR Path="/53A47160/53A47B1F" Ref="R2"  Part="1" 
AR Path="/53A4A568/53A49A26/53A47B1F" Ref="R2"  Part="1" 
F 0 "R2" V 4830 2900 40  0000 C CNN
F 1 "R" V 4757 2901 40  0000 C CNN
F 2 "~" V 4680 2900 30  0000 C CNN
F 3 "~" H 4750 2900 30  0000 C CNN
	1    4750 2900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5500 3000 5500 3150
Wire Wire Line
	5050 4950 5050 4800
Wire Wire Line
	5350 4800 5350 4900
Wire Wire Line
	5050 5350 5050 5450
Wire Wire Line
	5050 5450 5650 5450
Wire Wire Line
	5350 5450 5350 5300
Wire Wire Line
	6300 4350 6500 4350
Wire Wire Line
	6500 4750 6500 5250
Wire Wire Line
	5650 5250 6800 5250
Wire Wire Line
	5650 5450 5650 5250
Connection ~ 5350 5450
Wire Wire Line
	4500 2900 4500 2700
Wire Wire Line
	4900 3150 4900 2900
Wire Wire Line
	4900 2900 5100 2900
Wire Wire Line
	5200 2700 5000 2700
Wire Wire Line
	5050 2700 5050 3150
Wire Wire Line
	6800 5250 6800 2600
Wire Wire Line
	6800 2600 5500 2600
Connection ~ 6500 5250
Wire Wire Line
	4500 2700 4450 2700
Wire Wire Line
	5100 2900 5100 1800
Connection ~ 5000 2900
Wire Wire Line
	5200 2050 5200 2700
Connection ~ 5050 2700
Wire Wire Line
	6300 3600 6300 3100
Connection ~ 6800 3100
Wire Wire Line
	7650 3100 7650 3300
Wire Wire Line
	7650 3300 7500 3300
Text Label 4450 2700 0    60   ~ 0
vcc
Text Label 6800 3900 0    60   ~ 0
gnd
Text Label 7650 3100 0    60   ~ 0
gnd
Wire Wire Line
	6300 3100 6800 3100
Wire Wire Line
	6800 3100 6800 3050
Connection ~ 6800 3050
Text Label 5050 4900 0    60   ~ 0
vcc
Text Label 6500 4350 0    60   ~ 0
vcc
Text HLabel 3900 2700 0    60   Input ~ 0
SDA
Text HLabel 3900 2900 0    60   Input ~ 0
SCL
Wire Wire Line
	5100 1800 3900 1800
Wire Wire Line
	3900 1800 3900 2700
Wire Wire Line
	3900 2900 4050 2900
Wire Wire Line
	4050 2900 4050 2050
Wire Wire Line
	4050 2050 5200 2050
Text Label 7500 3450 0    60   ~ 0
vcc
$EndSCHEMATC
