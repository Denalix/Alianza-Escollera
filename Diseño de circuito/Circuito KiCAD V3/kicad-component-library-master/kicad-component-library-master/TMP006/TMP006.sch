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
LIBS:TMP00
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "22 jun 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L TMP U?
U 1 1 53A65A8D
P 5700 3650
F 0 "U?" H 5700 3920 60  0000 C CNN
F 1 "TMP" H 5710 4000 60  0000 C CNN
F 2 "" H 5700 3650 60  0000 C CNN
F 3 "" H 5700 3650 60  0000 C CNN
	1    5700 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 3700 5200 3750
Wire Wire Line
	6200 3700 6800 3700
Wire Wire Line
	6200 3750 6800 3750
$Comp
L R R?
U 1 1 53A65AA4
P 6300 4100
F 0 "R?" V 6380 4100 40  0000 C CNN
F 1 "R" V 6307 4101 40  0000 C CNN
F 2 "~" V 6230 4100 30  0000 C CNN
F 3 "~" H 6300 4100 30  0000 C CNN
	1    6300 4100
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53A65AB1
P 6500 4100
F 0 "R?" V 6580 4100 40  0000 C CNN
F 1 "R" V 6507 4101 40  0000 C CNN
F 2 "~" V 6430 4100 30  0000 C CNN
F 3 "~" H 6500 4100 30  0000 C CNN
	1    6500 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3850 6300 3750
Connection ~ 6300 3750
Wire Wire Line
	6500 3850 6500 3700
Connection ~ 6500 3700
Wire Wire Line
	6300 4350 6500 4350
Wire Wire Line
	6400 4350 6400 4500
Connection ~ 6400 4350
Wire Wire Line
	6200 3650 6800 3650
Wire Wire Line
	6200 3600 6800 3600
Wire Wire Line
	6200 3550 6800 3550
$Comp
L R R?
U 1 1 53A65B0E
P 6500 3150
F 0 "R?" V 6580 3150 40  0000 C CNN
F 1 "R" V 6507 3151 40  0000 C CNN
F 2 "~" V 6430 3150 30  0000 C CNN
F 3 "~" H 6500 3150 30  0000 C CNN
	1    6500 3150
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53A65B14
P 6700 3150
F 0 "R?" V 6780 3150 40  0000 C CNN
F 1 "R" V 6707 3151 40  0000 C CNN
F 2 "~" V 6630 3150 30  0000 C CNN
F 3 "~" H 6700 3150 30  0000 C CNN
	1    6700 3150
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53A65B1A
P 6300 3150
F 0 "R?" V 6380 3150 40  0000 C CNN
F 1 "R" V 6307 3151 40  0000 C CNN
F 2 "~" V 6230 3150 30  0000 C CNN
F 3 "~" H 6300 3150 30  0000 C CNN
	1    6300 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3400 6300 3550
Connection ~ 6300 3550
Wire Wire Line
	6500 3400 6500 3600
Connection ~ 6500 3600
Wire Wire Line
	6700 3400 6700 3650
Connection ~ 6700 3650
Wire Wire Line
	6300 2900 6700 2900
Connection ~ 6500 2900
$Comp
L 7PIN U?
U 1 1 53A65D07
P 7950 3600
F 0 "U?" H 7950 4200 60  0000 C CNN
F 1 "7PIN" H 7950 4100 60  0000 C CNN
F 2 "" H 7950 3600 60  0000 C CNN
F 3 "" H 7950 3600 60  0000 C CNN
	1    7950 3600
	1    0    0    -1  
$EndComp
$EndSCHEMATC
