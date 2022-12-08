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
Sheet 2 16
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
L ACS712 soic_81
U 1 1 53A5D884
P 5550 4150
AR Path="/53A5D878/53A5D884" Ref="soic_81"  Part="1" 
AR Path="/53A5DCF0/53A5D884" Ref="soic_82"  Part="1" 
AR Path="/53A7A60C/53A5D884" Ref="soic_82"  Part="1" 
AR Path="/53A82556/53A5D884" Ref="soic_82"  Part="1" 
AR Path="/53A82C62/53A5D884" Ref="soic_82"  Part="1" 
AR Path="/53A8339E/53A5D884" Ref="soic_82"  Part="1" 
AR Path="/53A8372C/53A5D884" Ref="soic_82"  Part="1" 
F 0 "soic_82" H 5900 4500 60  0000 C CNN
F 1 "ACS712" H 5550 4500 60  0000 C CNN
F 2 "~" H 5550 4150 60  0000 C CNN
F 3 "~" H 5550 4150 60  0000 C CNN
	1    5550 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4050 5050 3950
Wire Wire Line
	5050 4250 5050 4350
Text HLabel 6150 4650 0    60   Input ~ 0
gnd
Text HLabel 6300 3850 2    60   Input ~ 0
vcc
Text HLabel 6450 4150 2    60   Input ~ 0
vout
Text HLabel 4850 4000 0    60   Input ~ 0
IP+
Text HLabel 4850 4250 0    60   Input ~ 0
IP-
Wire Wire Line
	4850 4000 5050 4000
Connection ~ 5050 4000
Wire Wire Line
	5050 4250 4850 4250
Wire Wire Line
	6100 3750 6100 3950
Wire Wire Line
	6100 4350 6100 4550
Wire Wire Line
	5900 4550 6150 4550
Wire Wire Line
	6150 4550 6150 4650
Wire Wire Line
	6450 4150 6150 4150
Wire Wire Line
	6150 4150 6150 4050
Wire Wire Line
	6150 4050 6100 4050
$Comp
L C C2
U 1 1 53A5D8F3
P 6350 4450
AR Path="/53A5D878/53A5D8F3" Ref="C2"  Part="1" 
AR Path="/53A5DCF0/53A5D8F3" Ref="C4"  Part="1" 
AR Path="/53A7A60C/53A5D8F3" Ref="C4"  Part="1" 
AR Path="/53A82556/53A5D8F3" Ref="C4"  Part="1" 
AR Path="/53A82C62/53A5D8F3" Ref="C4"  Part="1" 
AR Path="/53A8339E/53A5D8F3" Ref="C4"  Part="1" 
AR Path="/53A8372C/53A5D8F3" Ref="C4"  Part="1" 
F 0 "C4" H 6350 4550 40  0000 L CNN
F 1 "1nf" H 6356 4365 39  0000 L CNN
F 2 "SM0603_Capa" H 6388 4300 30  0000 C CNN
F 3 "~" H 6350 4450 60  0000 C CNN
	1    6350 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4250 6350 4250
Text Label 5900 4550 0    60   ~ 0
GND
Connection ~ 6100 4550
Text Label 6100 3750 0    60   ~ 0
VCC
Wire Wire Line
	6100 3850 6300 3850
Connection ~ 6100 3850
Text Label 6350 4650 0    60   ~ 0
GND
Text Label 6850 4250 0    60   ~ 0
GND
$Comp
L C C3
U 1 1 53A5D9CB
P 6850 4050
AR Path="/53A5D878/53A5D9CB" Ref="C3"  Part="1" 
AR Path="/53A5DCF0/53A5D9CB" Ref="C5"  Part="1" 
AR Path="/53A7A60C/53A5D9CB" Ref="C5"  Part="1" 
AR Path="/53A82556/53A5D9CB" Ref="C5"  Part="1" 
AR Path="/53A82C62/53A5D9CB" Ref="C5"  Part="1" 
AR Path="/53A8339E/53A5D9CB" Ref="C5"  Part="1" 
AR Path="/53A8372C/53A5D9CB" Ref="C5"  Part="1" 
F 0 "C5" H 6850 4150 40  0000 L CNN
F 1 "0.1uf" H 6856 3965 40  0000 L CNN
F 2 "SM0603_Capa" H 6888 3900 30  0000 C CNN
F 3 "~" H 6850 4050 60  0000 C CNN
	1    6850 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 3950 6700 3950
Wire Wire Line
	6700 3950 6700 3850
Wire Wire Line
	6700 3850 6850 3850
Text Notes 5150 3350 0    197  ~ 0
ACS712
$EndSCHEMATC
