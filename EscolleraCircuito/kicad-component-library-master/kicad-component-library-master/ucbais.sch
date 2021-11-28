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
LIBS:PDM
LIBS:MAX485
LIBS:DAQ
LIBS:$$kk-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 16
Title "uc biAS"
Date "23 jun 2014"
Rev "100"
Comp ""
Comment1 "prototype module"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L SW_PUSH SW?
U 1 1 53A5EA3E
P 2850 3350
F 0 "SW?" H 3000 3460 50  0000 C CNN
F 1 "SW_PUSH" H 2850 3270 50  0000 C CNN
F 2 "~" H 2850 3350 60  0000 C CNN
F 3 "~" H 2850 3350 60  0000 C CNN
	1    2850 3350
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 53A5EA57
P 2850 2600
F 0 "R?" V 2930 2600 40  0000 C CNN
F 1 "10k" V 2857 2601 40  0000 C CNN
F 2 "~" V 2780 2600 30  0000 C CNN
F 3 "~" H 2850 2600 30  0000 C CNN
	1    2850 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3050 2850 2850
Wire Wire Line
	2850 2950 3150 2950
Connection ~ 2850 2950
Wire Wire Line
	2850 2350 2850 2300
Wire Wire Line
	2850 3650 2850 3750
$Comp
L C C?
U 1 1 53A5EA7B
P 5650 2800
F 0 "C?" H 5650 2900 40  0000 L CNN
F 1 "C" H 5656 2715 40  0000 L CNN
F 2 "~" H 5688 2650 30  0000 C CNN
F 3 "~" H 5650 2800 60  0000 C CNN
	1    5650 2800
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 53A5EA8A
P 5650 3400
F 0 "C?" H 5650 3500 40  0000 L CNN
F 1 "C" H 5656 3315 40  0000 L CNN
F 2 "~" H 5688 3250 30  0000 C CNN
F 3 "~" H 5650 3400 60  0000 C CNN
	1    5650 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5450 2800 5450 3400
$Comp
L CRYSTAL X?
U 1 1 53A5EAB0
P 5950 3100
F 0 "X?" H 5950 3250 60  0000 C CNN
F 1 "CRYSTAL" H 5950 2950 60  0000 C CNN
F 2 "~" H 5950 3100 60  0000 C CNN
F 3 "~" H 5950 3100 60  0000 C CNN
	1    5950 3100
	0    -1   1    0   
$EndComp
Wire Wire Line
	5850 3400 6350 3400
Wire Wire Line
	5850 2800 6350 2800
Text HLabel 6350 2800 2    197  Input ~ 0
xtal1
Text HLabel 6350 3400 2    197  Input ~ 0
xtal2
Connection ~ 5950 2800
Connection ~ 5950 3400
Text HLabel 9300 1600 0    197  Input ~ 0
gnd
Text Label 2850 3750 0    49   ~ 0
gnd
Text Label 5250 3100 0    49   ~ 0
gnd
Text Label 10000 1600 0    49   ~ 0
gnd
Wire Wire Line
	5250 3100 5450 3100
Connection ~ 5450 3100
Text Label 2850 2300 0    49   ~ 0
vcc
Wire Wire Line
	9300 1600 10000 1600
Text HLabel 3150 2950 2    49   Input ~ 0
reset
Wire Notes Line
	2350 4000 3500 4000
Wire Notes Line
	3500 4000 3500 2000
Wire Notes Line
	3500 2000 2350 2000
Wire Notes Line
	2350 2000 2350 4000
Wire Notes Line
	5100 4000 5100 2000
Wire Notes Line
	5100 2000 7700 2000
Wire Notes Line
	7700 2000 7700 4000
Wire Notes Line
	7700 4000 5100 4000
$Comp
L AVR-ISP-10 CON?
U 1 1 53A5EE05
P 4000 5600
F 0 "CON?" H 3830 5930 50  0000 C CNN
F 1 "AVR-ISP-10" H 3660 5270 50  0000 L BNN
F 2 "AVR-ISP-10" V 3250 5650 50  0001 C CNN
F 3 "" H 4000 5600 60  0000 C CNN
	1    4000 5600
	1    0    0    -1  
$EndComp
Text Label 4500 5700 0    49   ~ 0
gnd
Text Label 4050 5200 0    49   ~ 0
vcc
Text HLabel 2800 5350 0    49   Input ~ 0
MOSI
Text HLabel 2800 5500 0    49   Input ~ 0
MISO
Text HLabel 2800 5650 0    49   Input ~ 0
RST
Text HLabel 2800 5800 0    49   Input ~ 0
SCK
Wire Wire Line
	3800 5400 3500 5400
Wire Wire Line
	3500 5400 3500 5350
Wire Wire Line
	3500 5350 2800 5350
Wire Wire Line
	2800 5500 3350 5500
Wire Wire Line
	3350 5500 3350 5800
Wire Wire Line
	3350 5800 3800 5800
Wire Wire Line
	2800 5650 3550 5650
Wire Wire Line
	3550 5650 3550 5600
Wire Wire Line
	3550 5600 3800 5600
Wire Wire Line
	2800 5800 3250 5800
Wire Wire Line
	3250 5800 3250 5700
Wire Wire Line
	3250 5700 3800 5700
Wire Wire Line
	4050 5500 4050 5800
Connection ~ 4050 5700
Connection ~ 4050 5600
Wire Wire Line
	4050 5400 4050 5200
Wire Wire Line
	4050 5700 4500 5700
Wire Notes Line
	2350 6350 2350 4950
Wire Notes Line
	2350 4950 4750 4950
Wire Notes Line
	4750 4950 4750 6350
Wire Notes Line
	4750 6350 2350 6350
$Comp
L MY7805 so89?
U 1 1 53A69DD4
P 7450 5500
F 0 "so89?" H 7550 5700 60  0000 C CNN
F 1 "MY7805" H 7500 5600 60  0000 C CNN
F 2 "~" H 7450 5500 60  0000 C CNN
F 3 "~" H 7450 5500 60  0000 C CNN
	1    7450 5500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 53A69DE3
P 7050 5950
F 0 "C?" H 7050 6050 40  0000 L CNN
F 1 "100uf" H 7056 5865 40  0000 L CNN
F 2 "~" H 7088 5800 30  0000 C CNN
F 3 "~" H 7050 5950 60  0000 C CNN
	1    7050 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 5750 7300 5750
Wire Wire Line
	7050 6150 7850 6150
Wire Wire Line
	7450 6150 7450 5750
$Comp
L C C?
U 1 1 53A69E7A
P 7850 5950
F 0 "C?" H 7850 6050 40  0000 L CNN
F 1 "10uf" H 7856 5865 40  0000 L CNN
F 2 "~" H 7888 5800 30  0000 C CNN
F 3 "~" H 7850 5950 60  0000 C CNN
	1    7850 5950
	1    0    0    -1  
$EndComp
Connection ~ 7450 6150
Wire Wire Line
	7600 5750 8200 5750
Text Label 7450 6150 0    49   ~ 0
gnd
Text HLabel 6700 5700 0    49   Input ~ 0
vccout
Wire Wire Line
	6700 5700 6950 5700
Wire Wire Line
	6950 5700 6950 5750
Connection ~ 7050 5750
Text HLabel 8200 5750 2    49   Input ~ 0
pwrin
Connection ~ 7850 5750
Text Label 7050 5750 0    49   ~ 0
vcc
Wire Notes Line
	6300 6350 6300 4950
Wire Notes Line
	6300 4950 8600 4950
Wire Notes Line
	8600 4950 8600 6350
Wire Notes Line
	8600 6350 6300 6350
$EndSCHEMATC
