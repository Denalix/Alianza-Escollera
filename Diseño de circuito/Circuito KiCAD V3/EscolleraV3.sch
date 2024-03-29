EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Regulator_Linear:LM7805_TO220 U2
U 1 1 616EEB39
P 3650 2750
F 0 "U2" H 3650 2992 50  0000 C CNN
F 1 "LM7805_TO220" H 3650 2901 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3650 2975 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 3650 2700 50  0001 C CNN
	1    3650 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 616EF701
P 2050 1950
F 0 "J1" H 1968 2167 50  0000 C CNN
F 1 "Conn_01x02" H 1968 2076 50  0000 C CNN
F 2 "bornera:bornera" H 2050 1950 50  0001 C CNN
F 3 "~" H 2050 1950 50  0001 C CNN
	1    2050 1950
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 616F0A57
P 3100 2100
F 0 "C2" H 3215 2146 50  0000 L CNN
F 1 "0.1uF" H 3215 2055 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 3138 1950 50  0001 C CNN
F 3 "~" H 3100 2100 50  0001 C CNN
	1    3100 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 616F197F
P 4100 2100
F 0 "C3" H 4215 2146 50  0000 L CNN
F 1 "0.1uF" H 4215 2055 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 4138 1950 50  0001 C CNN
F 3 "~" H 4100 2100 50  0001 C CNN
	1    4100 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 616F1AC0
P 3150 2900
F 0 "C1" H 3265 2946 50  0000 L CNN
F 1 "0.1uF" H 3265 2855 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 3188 2750 50  0001 C CNN
F 3 "~" H 3150 2900 50  0001 C CNN
	1    3150 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 616F1AFA
P 4100 2900
F 0 "C4" H 4215 2946 50  0000 L CNN
F 1 "0.1uF" H 4215 2855 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 4138 2750 50  0001 C CNN
F 3 "~" H 4100 2900 50  0001 C CNN
	1    4100 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1950 4000 1950
Wire Wire Line
	4100 2300 4100 2250
Wire Wire Line
	3700 2250 3700 2300
Connection ~ 3700 2300
Wire Wire Line
	3700 2300 4100 2300
Wire Wire Line
	3150 2750 3350 2750
Wire Wire Line
	3950 2750 4100 2750
Wire Wire Line
	4100 3050 4100 3100
Wire Wire Line
	3150 3100 3150 3050
Wire Wire Line
	3650 3050 3650 3100
Connection ~ 3650 3100
$Comp
L power:GND #PWR0101
U 1 1 61704DE4
P 2650 2400
F 0 "#PWR0101" H 2650 2150 50  0001 C CNN
F 1 "GND" H 2655 2227 50  0000 C CNN
F 2 "" H 2650 2400 50  0001 C CNN
F 3 "" H 2650 2400 50  0001 C CNN
	1    2650 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2300 2650 2400
Connection ~ 2650 2300
$Comp
L Interface_UART:MAX485E U4
U 1 1 616E2D0F
P 5750 2250
F 0 "U4" V 5796 1606 50  0000 R CNN
F 1 "MAX485E" V 5705 1606 50  0000 R CNN
F 2 "escolleraHuellas:max485_TTL_RS485" H 5750 1550 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX1487E-MAX491E.pdf" H 5750 2300 50  0001 C CNN
	1    5750 2250
	0    -1   -1   0   
$EndComp
$Comp
L TTGO_Tbeam:ttgo_tbeam U3
U 1 1 62E821CD
P 4000 4150
F 0 "U3" V 3954 4878 50  0000 L CNN
F 1 "ttgo_tbeam" V 4045 4878 50  0000 L CNN
F 2 "ttgo_tbeam:ttgo_tbeam" V 4091 4878 50  0001 L CNN
F 3 "" H 4000 4350 50  0001 C CNN
	1    4000 4150
	0    1    1    0   
$EndComp
$Comp
L Device:CP C5
U 1 1 62E89CBB
P 2800 2100
F 0 "C5" H 2918 2146 50  0000 L CNN
F 1 "0.22uF" H 2918 2055 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D5.0mm_H11.0mm_P2.00mm" H 2838 1950 50  0001 C CNN
F 3 "~" H 2800 2100 50  0001 C CNN
	1    2800 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2300 2800 2300
Wire Wire Line
	2800 2250 2800 2300
Connection ~ 2800 2300
Wire Wire Line
	2800 2300 3100 2300
Wire Wire Line
	3100 2250 3100 2300
Connection ~ 3100 2300
Wire Wire Line
	3100 2300 3700 2300
Wire Wire Line
	2800 1950 3100 1950
Wire Wire Line
	3100 1950 3400 1950
Connection ~ 3100 1950
$Comp
L Device:CP C7
U 1 1 62EAA159
P 4550 2100
F 0 "C7" H 4668 2146 50  0000 L CNN
F 1 "10uF" H 4668 2055 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D5.0mm_H11.0mm_P2.00mm" H 4588 1950 50  0001 C CNN
F 3 "~" H 4550 2100 50  0001 C CNN
	1    4550 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1950 4350 1950
Connection ~ 4100 1950
Wire Wire Line
	4550 2300 4550 2250
Connection ~ 4100 2300
Wire Wire Line
	3650 3100 4100 3100
$Comp
L Device:CP C6
U 1 1 62EBD6DD
P 2850 2900
F 0 "C6" H 2968 2946 50  0000 L CNN
F 1 "0.22uF" H 2968 2855 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D5.0mm_H11.0mm_P2.00mm" H 2888 2750 50  0001 C CNN
F 3 "~" H 2850 2900 50  0001 C CNN
	1    2850 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C8
U 1 1 62EBDEB3
P 4550 2900
F 0 "C8" H 4668 2946 50  0000 L CNN
F 1 "10uF" H 4668 2855 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D5.0mm_H11.0mm_P2.00mm" H 4588 2750 50  0001 C CNN
F 3 "~" H 4550 2900 50  0001 C CNN
	1    4550 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 2750 4100 2750
Connection ~ 4100 2750
Wire Wire Line
	4550 3050 4550 3100
Wire Wire Line
	4550 3100 4100 3100
Connection ~ 4100 3100
Wire Wire Line
	3150 3100 2850 3100
Wire Wire Line
	2850 3100 2850 3050
Connection ~ 3150 3100
Wire Wire Line
	2850 2750 3150 2750
Connection ~ 3150 2750
Wire Wire Line
	2650 2300 2450 2300
Wire Wire Line
	2450 2300 2450 3100
Connection ~ 2850 3100
Wire Wire Line
	4550 1950 4850 1950
Wire Wire Line
	4850 1950 4850 2400
Wire Wire Line
	4850 2400 2850 2400
Wire Wire Line
	2850 2400 2850 2750
Connection ~ 4550 1950
Connection ~ 2850 2750
Wire Wire Line
	2250 2050 2650 2050
Wire Wire Line
	2650 2050 2650 2300
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 62F0642C
P 7250 3050
F 0 "J2" H 7330 3042 50  0000 L CNN
F 1 "Conn_01x02" H 7330 2951 50  0000 L CNN
F 2 "bornera:bornera" H 7250 3050 50  0001 C CNN
F 3 "~" H 7250 3050 50  0001 C CNN
	1    7250 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 62F069C6
P 7250 3250
F 0 "J3" H 7330 3242 50  0000 L CNN
F 1 "Conn_01x02" H 7330 3151 50  0000 L CNN
F 2 "bornera:bornera" H 7250 3250 50  0001 C CNN
F 3 "~" H 7250 3250 50  0001 C CNN
	1    7250 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 62F06C66
P 7250 3700
F 0 "J4" H 7330 3692 50  0000 L CNN
F 1 "Conn_01x02" H 7330 3601 50  0000 L CNN
F 2 "bornera:bornera" H 7250 3700 50  0001 C CNN
F 3 "~" H 7250 3700 50  0001 C CNN
	1    7250 3700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 62F071C7
P 7250 3900
F 0 "J5" H 7330 3892 50  0000 L CNN
F 1 "Conn_01x02" H 7330 3801 50  0000 L CNN
F 2 "bornera:bornera" H 7250 3900 50  0001 C CNN
F 3 "~" H 7250 3900 50  0001 C CNN
	1    7250 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3250 6900 3250
Wire Wire Line
	6550 3500 6550 1600
Wire Wire Line
	5950 1850 6650 1850
Wire Wire Line
	6650 1850 6650 3250
NoConn ~ 3400 4650
NoConn ~ 3500 4650
NoConn ~ 3600 4650
NoConn ~ 3700 4650
NoConn ~ 3900 4650
NoConn ~ 4000 4650
NoConn ~ 4100 4650
NoConn ~ 4200 4650
NoConn ~ 4300 4650
NoConn ~ 4000 3650
NoConn ~ 4100 3650
NoConn ~ 4200 3650
NoConn ~ 4300 3650
NoConn ~ 4400 3650
NoConn ~ 4500 3650
NoConn ~ 4600 3650
Wire Wire Line
	6800 3150 7050 3150
Wire Wire Line
	6900 3900 7050 3900
Wire Wire Line
	6800 3800 7050 3800
Wire Wire Line
	6900 3250 6900 3500
Wire Wire Line
	6800 3150 6800 3250
Wire Wire Line
	6650 3250 6800 3250
Connection ~ 6800 3250
Wire Wire Line
	6800 3250 6800 3800
Wire Wire Line
	6900 3500 6550 3500
Connection ~ 6900 3500
Wire Wire Line
	6900 3500 6900 3900
$Comp
L Regulator_Linear:L7809 U1
U 1 1 62F212D2
P 3700 1950
F 0 "U1" H 3700 2192 50  0000 C CNN
F 1 "L7809" H 3700 2101 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3725 1800 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 3700 1900 50  0001 C CNN
	1    3700 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2300 4550 2300
Connection ~ 4350 1950
Wire Wire Line
	4350 1950 4100 1950
Wire Wire Line
	3150 3100 3500 3100
$Comp
L Diode:1N4004 D5
U 1 1 63656508
P 5550 3300
F 0 "D5" H 5550 3517 50  0000 C CNN
F 1 "1N4004" H 5550 3426 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P3.81mm_Vertical_AnodeUp" H 5550 3125 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 5550 3300 50  0001 C CNN
	1    5550 3300
	0    1    1    0   
$EndComp
$Comp
L Diode:1N4004 D3
U 1 1 63656DE0
P 4600 1250
F 0 "D3" H 4600 1467 50  0000 C CNN
F 1 "1N4004" H 4600 1376 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P3.81mm_Vertical_AnodeUp" H 4600 1075 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 4600 1250 50  0001 C CNN
	1    4600 1250
	0    1    1    0   
$EndComp
$Comp
L Diode:1N4004 D1
U 1 1 63657516
P 2550 1950
F 0 "D1" H 2550 1733 50  0000 C CNN
F 1 "1N4004" H 2550 1824 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P3.81mm_Vertical_AnodeUp" H 2550 1775 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 2550 1950 50  0001 C CNN
	1    2550 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	2250 1950 2400 1950
Wire Wire Line
	2700 1950 2800 1950
Connection ~ 2800 1950
$Comp
L Connector_Generic:Conn_01x02 J8
U 1 1 6365B5EB
P 5150 1200
F 0 "J8" H 5230 1192 50  0000 L CNN
F 1 "Conn_01x02" H 5230 1101 50  0000 L CNN
F 2 "bornera:bornera" H 5150 1200 50  0001 C CNN
F 3 "~" H 5150 1200 50  0001 C CNN
	1    5150 1200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J9
U 1 1 6365B5F1
P 6000 3250
F 0 "J9" H 6080 3242 50  0000 L CNN
F 1 "Conn_01x02" H 6080 3151 50  0000 L CNN
F 2 "bornera:bornera" H 6000 3250 50  0001 C CNN
F 3 "~" H 6000 3250 50  0001 C CNN
	1    6000 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J6
U 1 1 6365D31F
P 2500 3850
F 0 "J6" H 2580 3842 50  0000 L CNN
F 1 "Conn_01x02" H 2580 3751 50  0000 L CNN
F 2 "bornera:bornera" H 2500 3850 50  0001 C CNN
F 3 "~" H 2500 3850 50  0001 C CNN
	1    2500 3850
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 6365D325
P 2500 4050
F 0 "J7" H 2580 4042 50  0000 L CNN
F 1 "Conn_01x02" H 2580 3951 50  0000 L CNN
F 2 "bornera:bornera" H 2500 4050 50  0001 C CNN
F 3 "~" H 2500 4050 50  0001 C CNN
	1    2500 4050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4950 1200 4950 1050
Wire Wire Line
	4950 1050 4600 1050
Wire Wire Line
	4600 1050 4600 1100
Wire Wire Line
	4600 1400 4600 1450
Wire Wire Line
	4600 1450 4950 1450
Wire Wire Line
	4950 1450 4950 1300
Wire Wire Line
	4350 1050 4350 1950
Connection ~ 4600 1050
Wire Wire Line
	4550 2750 4900 2750
Connection ~ 4550 2750
Wire Wire Line
	3500 3650 3500 3100
Connection ~ 3500 3100
Wire Wire Line
	3500 3100 3650 3100
$Comp
L Device:LED D4
U 1 1 636797A2
P 5150 1550
F 0 "D4" H 5150 1450 50  0000 C CNN
F 1 "LED_amarillo" H 5150 1650 50  0000 C CNN
F 2 "LED_THT:LED_D3.0mm" H 5150 1550 50  0001 C CNN
F 3 "~" H 5150 1550 50  0001 C CNN
	1    5150 1550
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D2
U 1 1 6367A0A8
P 3000 3450
F 0 "D2" H 2993 3667 50  0000 C CNN
F 1 "LED_rojo" H 2993 3576 50  0000 C CNN
F 2 "LED_THT:LED_D3.0mm" H 3000 3450 50  0001 C CNN
F 3 "~" H 3000 3450 50  0001 C CNN
	1    3000 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D6
U 1 1 6367A448
P 6000 3800
F 0 "D6" H 5993 4017 50  0000 C CNN
F 1 "LED_azul" H 5993 3926 50  0000 C CNN
F 2 "LED_THT:LED_D3.0mm" H 6000 3800 50  0001 C CNN
F 3 "~" H 6000 3800 50  0001 C CNN
	1    6000 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 6367B92B
P 5550 3800
F 0 "R6" V 5343 3800 50  0000 C CNN
F 1 "330" V 5434 3800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5480 3800 50  0001 C CNN
F 3 "~" H 5550 3800 50  0001 C CNN
	1    5550 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 1850 5650 1600
Wire Wire Line
	5650 1600 6550 1600
Wire Wire Line
	5800 3250 5800 3100
Wire Wire Line
	5800 3100 5550 3100
Wire Wire Line
	5550 3100 5550 3150
Wire Wire Line
	5800 3350 5800 3500
Wire Wire Line
	5800 3500 5550 3500
Wire Wire Line
	5550 3500 5550 3450
Wire Wire Line
	5550 3050 5550 3100
Connection ~ 5550 3100
Wire Wire Line
	5800 3500 6150 3500
Wire Wire Line
	6150 3500 6150 3000
Wire Wire Line
	5100 2250 5250 2250
Connection ~ 5800 3500
Wire Wire Line
	4350 1050 4600 1050
Wire Wire Line
	5300 1550 5300 1850
Wire Wire Line
	4550 2300 4950 2300
Wire Wire Line
	4950 2300 4950 2150
Wire Wire Line
	4950 1850 5000 1850
Connection ~ 4550 2300
Wire Wire Line
	4950 1450 5000 1450
Wire Wire Line
	5000 1450 5000 1550
Connection ~ 4950 1450
Wire Wire Line
	5700 3800 5850 3800
Wire Wire Line
	6150 3500 6150 3800
Connection ~ 6150 3500
$Comp
L Device:R R4
U 1 1 636F246F
P 2500 3450
F 0 "R4" V 2293 3450 50  0000 C CNN
F 1 "330" V 2384 3450 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2430 3450 50  0001 C CNN
F 3 "~" H 2500 3450 50  0001 C CNN
	1    2500 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	3150 3450 3200 3450
Wire Wire Line
	2850 3450 2650 3450
Wire Wire Line
	2350 3450 2350 3150
Wire Wire Line
	2350 3100 2450 3100
Connection ~ 2450 3100
Wire Wire Line
	2450 3100 2850 3100
Wire Wire Line
	4950 2750 4950 3350
Wire Wire Line
	3400 3350 4950 3350
Wire Wire Line
	5100 2250 5100 3000
Wire Wire Line
	5100 3000 6150 3000
Wire Wire Line
	5000 3050 5000 2750
Wire Wire Line
	5000 2750 4950 2750
Wire Wire Line
	5000 3050 5550 3050
Connection ~ 4950 2750
Wire Wire Line
	5750 2650 5750 2700
Wire Wire Line
	5750 2700 5800 2700
Wire Wire Line
	5850 2700 5850 2650
Wire Wire Line
	4500 4650 4500 4900
Wire Wire Line
	4500 4900 6250 4900
Wire Wire Line
	6250 4900 6250 2850
Wire Wire Line
	6250 2850 5650 2850
Wire Wire Line
	5650 2850 5650 2650
Wire Wire Line
	5950 2650 5950 2800
Wire Wire Line
	5950 2800 6300 2800
Wire Wire Line
	6300 2800 6300 4950
Wire Wire Line
	6300 4950 4600 4950
Wire Wire Line
	4600 4950 4600 4650
Wire Wire Line
	5800 2700 5800 2750
Wire Wire Line
	5800 2750 6500 2750
Wire Wire Line
	6500 2750 6500 4000
Wire Wire Line
	6500 4000 5000 4000
Wire Wire Line
	5000 4000 5000 3400
Wire Wire Line
	5000 3400 3700 3400
Wire Wire Line
	3700 3400 3700 3650
Connection ~ 5800 2700
Wire Wire Line
	5800 2700 5850 2700
NoConn ~ 3800 4650
NoConn ~ 4400 4650
Wire Wire Line
	2700 4150 3200 4150
Wire Wire Line
	3200 4150 3200 3450
Wire Wire Line
	2350 3150 2250 3150
Wire Wire Line
	2250 3150 2250 3500
Wire Wire Line
	2250 3700 2950 3700
Wire Wire Line
	2950 3700 2950 3850
Wire Wire Line
	2950 3850 2700 3850
Connection ~ 2350 3150
Wire Wire Line
	2350 3150 2350 3100
Wire Wire Line
	3800 3650 3800 3600
Wire Wire Line
	3800 3600 3150 3600
Wire Wire Line
	3150 3600 3150 3950
Wire Wire Line
	3150 3950 2700 3950
Wire Wire Line
	3900 3650 3900 3550
Wire Wire Line
	3900 3550 3100 3550
Wire Wire Line
	3100 3550 3100 4050
Wire Wire Line
	3100 4050 2700 4050
$Comp
L Device:R R1
U 1 1 63765878
P 1900 2550
F 0 "R1" H 1970 2596 50  0000 L CNN
F 1 "10K" H 1970 2505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1830 2550 50  0001 C CNN
F 3 "~" H 1900 2550 50  0001 C CNN
	1    1900 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 63765B43
P 1900 2950
F 0 "R2" H 1970 2996 50  0000 L CNN
F 1 "2K" H 1970 2905 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1830 2950 50  0001 C CNN
F 3 "~" H 1900 2950 50  0001 C CNN
	1    1900 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 63765CDF
P 1900 3300
F 0 "R3" H 1970 3346 50  0000 L CNN
F 1 "1K" H 1970 3255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1830 3300 50  0001 C CNN
F 3 "~" H 1900 3300 50  0001 C CNN
	1    1900 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 1950 2400 2250
Wire Wire Line
	2400 2250 1900 2250
Wire Wire Line
	1900 2250 1900 2400
Connection ~ 2400 1950
Wire Wire Line
	1900 2700 1900 2750
Wire Wire Line
	1900 3100 1900 3150
Wire Wire Line
	1900 3450 1900 3500
Wire Wire Line
	1900 3500 2250 3500
Connection ~ 2250 3500
Wire Wire Line
	2250 3500 2250 3700
Wire Wire Line
	3600 3650 3600 3300
Wire Wire Line
	3600 3300 2100 3300
Wire Wire Line
	2100 3300 2100 2750
Wire Wire Line
	2100 2750 1900 2750
Connection ~ 1900 2750
Wire Wire Line
	1900 2750 1900 2800
$Comp
L Device:R R5
U 1 1 6367AF59
P 5150 1850
F 0 "R5" V 5250 1850 50  0000 C CNN
F 1 "330" V 5050 1850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5080 1850 50  0001 C CNN
F 3 "~" H 5150 1850 50  0001 C CNN
	1    5150 1850
	0    1    -1   0   
$EndComp
Wire Wire Line
	6350 2250 6800 2250
Wire Wire Line
	6800 2250 6800 1550
Wire Wire Line
	6800 1550 5450 1550
Wire Wire Line
	5450 1550 5450 1900
Wire Wire Line
	5450 1900 5300 1900
Wire Wire Line
	5300 1900 5300 2150
Wire Wire Line
	5300 2150 4950 2150
Connection ~ 4950 2150
Wire Wire Line
	4950 2150 4950 1850
Wire Wire Line
	3400 3350 3400 3650
Wire Wire Line
	4900 2750 4900 3250
Wire Wire Line
	4900 3250 3200 3250
Wire Wire Line
	3200 3250 3200 3450
Connection ~ 4900 2750
Wire Wire Line
	4900 2750 4950 2750
Connection ~ 3200 3450
Text Label 2700 4150 0    50   ~ 0
5V_f
Text Label 2700 4050 0    50   ~ 0
IN2
Text Label 2700 3950 0    50   ~ 0
IN1
Text Label 2700 3850 0    50   ~ 0
GND
Text Label 7000 3150 0    50   ~ 0
A
Text Label 7000 3050 0    50   ~ 0
9V_s
Text Label 7000 3250 0    50   ~ 0
B
Text Label 7000 3800 0    50   ~ 0
A
Text Label 7000 3900 0    50   ~ 0
B
Text Label 7000 3700 0    50   ~ 0
9V_s
Wire Wire Line
	6900 4000 7050 4000
Text Label 6950 4000 0    50   ~ 0
GND
Text Label 6950 3350 0    50   ~ 0
GND
Wire Wire Line
	6950 3350 7050 3350
Wire Wire Line
	6950 3350 6950 3550
Wire Wire Line
	6950 3550 8000 3550
Wire Wire Line
	8000 3550 8000 3850
Wire Wire Line
	8000 4150 6900 4150
Wire Wire Line
	6900 4150 6900 4000
Wire Wire Line
	6800 2250 8200 2250
Wire Wire Line
	8200 2250 8200 3850
Wire Wire Line
	8200 3850 8000 3850
Connection ~ 6800 2250
Connection ~ 8000 3850
Wire Wire Line
	8000 3850 8000 4150
Wire Wire Line
	6850 3700 6850 3050
Wire Wire Line
	6850 3700 7050 3700
Wire Wire Line
	6850 3050 7050 3050
Wire Wire Line
	5000 1450 6850 1450
Wire Wire Line
	6850 1450 6850 3050
Connection ~ 5000 1450
Connection ~ 6850 3050
Wire Wire Line
	5200 3800 5200 3100
Wire Wire Line
	5200 3100 4550 3100
Wire Wire Line
	5200 3800 5400 3800
Connection ~ 4550 3100
Text Label 4000 1950 0    50   ~ 0
9V_f
Text Label 4000 2750 0    50   ~ 0
5V_f
Text Label 5100 2250 0    50   ~ 0
5V_m
$EndSCHEMATC
