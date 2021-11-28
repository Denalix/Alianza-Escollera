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
Sheet 4 16
Title "Prototype Module"
Date "23 jun 2014"
Rev "101"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MAX485 U?
U 1 1 53A5E28D
P 4850 4200
AR Path="/53A5E282/53A5E28D" Ref="U?"  Part="1" 
AR Path="/53AAD9C1/53A5E28D" Ref="U?"  Part="1" 
F 0 "U?" H 5150 4600 60  0000 C CNN
F 1 "MAX485" H 4900 4600 60  0000 C CNN
F 2 "~" H 4850 4200 60  0000 C CNN
F 3 "~" H 4850 4200 60  0000 C CNN
	1    4850 4200
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 53A5E29C
P 5950 4100
AR Path="/53A5E282/53A5E29C" Ref="R?"  Part="1" 
AR Path="/53AAD9C1/53A5E29C" Ref="R?"  Part="1" 
F 0 "R?" V 6030 4100 40  0000 C CNN
F 1 "R" V 5957 4101 40  0000 C CNN
F 2 "~" V 5880 4100 30  0000 C CNN
F 3 "~" H 5950 4100 30  0000 C CNN
	1    5950 4100
	1    0    0    1   
$EndComp
Wire Wire Line
	5450 4050 5750 4050
Wire Wire Line
	5750 4050 5750 3850
Wire Wire Line
	5750 3850 6150 3850
Wire Wire Line
	5750 4350 6150 4350
Wire Wire Line
	5750 4350 5750 4150
Wire Wire Line
	5750 4150 5450 4150
Text Label 6900 3300 0    60   ~ 0
gnd
Text Label 5450 4250 0    60   ~ 0
gnd
Text Label 6900 3150 0    60   ~ 0
vcc
Text Label 5450 3950 0    60   ~ 0
vcc
Text HLabel 4250 4250 0    60   Input ~ 0
tx
Text HLabel 4200 3950 0    60   Input ~ 0
rx
Text HLabel 3900 4100 0    60   Input ~ 0
data dir
Connection ~ 5950 4350
Connection ~ 5950 3850
Text HLabel 6800 3150 0    60   Input ~ 0
5vcc
Text HLabel 6800 3300 0    60   Input ~ 0
gnd
Wire Wire Line
	6800 3150 6900 3150
Wire Wire Line
	6800 3300 6900 3300
Wire Wire Line
	4250 4250 4450 4250
Wire Wire Line
	4450 3950 4200 3950
Wire Wire Line
	4450 4150 4450 4050
Wire Wire Line
	4450 4050 4150 4050
Wire Wire Line
	4150 4050 4150 4100
Wire Wire Line
	4150 4100 3900 4100
$Comp
L M485_TERMINAL U?
U 1 1 53A5E408
P 7250 4050
AR Path="/53A5E282/53A5E408" Ref="U?"  Part="1" 
AR Path="/53AAD9C1/53A5E408" Ref="U?"  Part="1" 
F 0 "U?" H 7700 4450 60  0000 C CNN
F 1 "M485_TERMINAL" H 7250 4450 60  0000 C CNN
F 2 "~" H 7250 4050 60  0000 C CNN
F 3 "~" H 7250 4050 60  0000 C CNN
	1    7250 4050
	-1   0    0    1   
$EndComp
Text HLabel 6150 4350 2    60   Input ~ 0
B
Text HLabel 6150 3850 2    60   Input ~ 0
A
Wire Wire Line
	6100 3850 6100 4000
Wire Wire Line
	6100 4000 6750 4000
Connection ~ 6100 3850
Wire Wire Line
	6100 4350 6100 4150
Wire Wire Line
	6100 4150 6750 4150
Connection ~ 6100 4350
Text Notes 4200 3350 0    197  ~ 0
MAX485\n
Text Label 8600 3600 0    49   ~ 0
vcc
Text Label 8600 3700 0    49   ~ 0
vcc
Text Label 8600 3800 0    49   ~ 0
vcc
Text Label 8600 3900 0    49   ~ 0
vcc
Text Label 8900 3000 0    49   ~ 0
pa0
Text Label 8900 3100 0    49   ~ 0
pa1
Text Label 8900 3200 0    49   ~ 0
pa2
Text Label 8900 3300 0    49   ~ 0
pa3
Text Label 8900 3400 0    49   ~ 0
pa4
Text Label 8900 3500 0    49   ~ 0
pa5
Text Label 8900 3600 0    49   ~ 0
pa6
Text Label 8900 3700 0    49   ~ 0
pa7
Text Label 8900 3800 0    49   ~ 0
pa8
$EndSCHEMATC
