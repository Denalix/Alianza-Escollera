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
L Interface_UART:MAX485E U3
U 1 1 616E446E
P 5750 2750
F 0 "U3" V 5796 2106 50  0000 R CNN
F 1 "MAX485E" V 5705 2106 50  0000 R CNN
F 2 "escolleraHuellas:max485_TTL_RS485" H 5750 2050 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX1487E-MAX491E.pdf" H 5750 2800 50  0001 C CNN
	1    5750 2750
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 616E634A
P 6950 3850
F 0 "J3" H 6868 3425 50  0000 C CNN
F 1 "Conn_01x04" H 6868 3516 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-396_A-41791-0004_1x04_P3.96mm_Vertical" H 6950 3850 50  0001 C CNN
F 3 "~" H 6950 3850 50  0001 C CNN
	1    6950 3850
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 616E77DA
P 6950 3200
F 0 "J2" H 6868 2775 50  0000 C CNN
F 1 "Conn_01x04" H 6868 2866 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-396_A-41791-0004_1x04_P3.96mm_Vertical" H 6950 3200 50  0001 C CNN
F 3 "~" H 6950 3200 50  0001 C CNN
	1    6950 3200
	1    0    0    1   
$EndComp
$Comp
L Regulator_Linear:LM7812_TO220 U1
U 1 1 616ED667
P 4250 2400
F 0 "U1" H 4250 2642 50  0000 C CNN
F 1 "LM7812_TO220" H 4250 2551 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4250 2625 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 4250 2350 50  0001 C CNN
	1    4250 2400
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM7805_TO220 U2
U 1 1 616EEB39
P 4250 3050
F 0 "U2" H 4250 3292 50  0000 C CNN
F 1 "LM7805_TO220" H 4250 3201 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4250 3275 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 4250 3000 50  0001 C CNN
	1    4250 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 616EF701
P 3250 2500
F 0 "J1" H 3168 2717 50  0000 C CNN
F 1 "Conn_01x02" H 3168 2626 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-396_A-41791-0002_1x02_P3.96mm_Vertical" H 3250 2500 50  0001 C CNN
F 3 "~" H 3250 2500 50  0001 C CNN
	1    3250 2500
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 616F0A57
P 3750 2550
F 0 "C2" H 3865 2596 50  0000 L CNN
F 1 "330nF" H 3865 2505 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 3788 2400 50  0001 C CNN
F 3 "~" H 3750 2550 50  0001 C CNN
	1    3750 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 616F197F
P 4650 2550
F 0 "C3" H 4765 2596 50  0000 L CNN
F 1 "100nF" H 4765 2505 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 4688 2400 50  0001 C CNN
F 3 "~" H 4650 2550 50  0001 C CNN
	1    4650 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 616F1AC0
P 3750 3200
F 0 "C1" H 3865 3246 50  0000 L CNN
F 1 "330nF" H 3865 3155 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 3788 3050 50  0001 C CNN
F 3 "~" H 3750 3200 50  0001 C CNN
	1    3750 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 616F1AFA
P 4700 3200
F 0 "C4" H 4815 3246 50  0000 L CNN
F 1 "100nF" H 4815 3155 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 4738 3050 50  0001 C CNN
F 3 "~" H 4700 3200 50  0001 C CNN
	1    4700 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2400 3750 2400
Wire Wire Line
	4650 2400 4550 2400
Wire Wire Line
	3750 2700 3750 2750
Wire Wire Line
	3750 2750 4250 2750
Wire Wire Line
	4650 2750 4650 2700
Wire Wire Line
	4250 2700 4250 2750
Connection ~ 4250 2750
Wire Wire Line
	4250 2750 4650 2750
Wire Wire Line
	3750 3050 3950 3050
Wire Wire Line
	4550 3050 4700 3050
Wire Wire Line
	4700 3350 4700 3400
Wire Wire Line
	4700 3400 4600 3400
Wire Wire Line
	3750 3400 3750 3350
Wire Wire Line
	4250 3350 4250 3400
Connection ~ 4250 3400
Wire Wire Line
	4250 3400 3750 3400
Wire Wire Line
	4650 2400 4650 2250
Wire Wire Line
	4650 2250 4750 2250
Wire Wire Line
	4750 2250 4750 2900
Wire Wire Line
	4750 2900 3750 2900
Wire Wire Line
	3750 2900 3750 3050
Connection ~ 4650 2400
Connection ~ 3750 3050
Wire Wire Line
	5000 3050 5000 2750
Wire Wire Line
	5000 2750 5250 2750
Connection ~ 4700 3050
Wire Wire Line
	5250 3700 5000 3700
Wire Wire Line
	5000 3700 5000 3050
Connection ~ 5000 3050
Wire Wire Line
	6350 2750 6400 2750
Wire Wire Line
	6400 2750 6400 3300
Wire Wire Line
	6400 3700 6350 3700
Wire Wire Line
	6400 3950 6400 3700
Connection ~ 6400 3700
Wire Wire Line
	6750 3300 6400 3300
Connection ~ 6400 3300
Wire Wire Line
	6400 3300 6400 3700
Wire Wire Line
	6700 2250 6700 3000
Wire Wire Line
	6700 3000 6750 3000
Connection ~ 4750 2250
Wire Wire Line
	6700 3000 6700 3650
Wire Wire Line
	6700 3650 6750 3650
Connection ~ 6700 3000
Wire Wire Line
	3450 2500 3450 2400
Wire Wire Line
	3450 2400 3750 2400
Connection ~ 3750 2400
Wire Wire Line
	3450 2600 3450 2750
Wire Wire Line
	3450 2750 3600 2750
Connection ~ 3750 2750
$Comp
L power:GND #PWR0101
U 1 1 61704DE4
P 3450 2850
F 0 "#PWR0101" H 3450 2600 50  0001 C CNN
F 1 "GND" H 3455 2677 50  0000 C CNN
F 2 "" H 3450 2850 50  0001 C CNN
F 3 "" H 3450 2850 50  0001 C CNN
	1    3450 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2750 3450 2850
Connection ~ 3450 2750
$Comp
L Escollera-rescue:TTGO_LoRa32-ttgo_lora32 U5
U 1 1 6170B131
P 4900 4150
F 0 "U5" V 4951 4222 50  0000 R CNN
F 1 "TTGO_LoRa32" V 4860 4222 50  0000 R CNN
F 2 "escolleraHuellas:TTGO" H 4900 4200 50  0001 C CNN
F 3 "" H 4900 4200 50  0001 C CNN
	1    4900 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4800 3050 4800 3550
Wire Wire Line
	4800 3550 4700 3550
Wire Wire Line
	4700 3550 4700 3700
Wire Wire Line
	4700 3050 4800 3050
Connection ~ 4800 3050
Wire Wire Line
	4800 3050 5000 3050
Wire Wire Line
	4600 3700 4600 3400
Connection ~ 4600 3400
Wire Wire Line
	4600 3400 4250 3400
Wire Wire Line
	3750 3400 3600 3400
Wire Wire Line
	3600 3400 3600 2750
Connection ~ 3750 3400
Connection ~ 3600 2750
Wire Wire Line
	3600 2750 3750 2750
Wire Wire Line
	4700 3400 5200 3400
Wire Wire Line
	5200 3400 5200 2300
Wire Wire Line
	5200 2300 6400 2300
Wire Wire Line
	6400 2300 6400 2750
Connection ~ 4700 3400
Connection ~ 6400 2750
Wire Wire Line
	5650 3150 5250 3150
Wire Wire Line
	5250 3150 5250 3500
Wire Wire Line
	5250 3500 3700 3500
Wire Wire Line
	3700 3500 3700 3700
Wire Wire Line
	5950 4100 5950 4250
Wire Wire Line
	5950 4250 3300 4250
Wire Wire Line
	3300 4250 3300 3500
Wire Wire Line
	3300 3500 3600 3500
Wire Wire Line
	3600 3500 3600 3700
Wire Wire Line
	5750 3150 5750 3250
Wire Wire Line
	5750 3250 5850 3250
Wire Wire Line
	5850 3250 5850 3150
Wire Wire Line
	4200 4600 4200 4800
Wire Wire Line
	4200 4800 5550 4800
Wire Wire Line
	5550 4800 5550 3250
Wire Wire Line
	5550 3250 5750 3250
Connection ~ 5750 3250
Wire Wire Line
	4100 4600 4100 4700
Wire Wire Line
	4100 4700 5750 4700
Wire Wire Line
	5750 4700 5750 4200
Wire Wire Line
	5850 4100 5850 4200
Wire Wire Line
	5850 4200 5750 4200
Connection ~ 5750 4200
Wire Wire Line
	5750 4200 5750 4100
NoConn ~ 3500 4600
NoConn ~ 3600 4600
NoConn ~ 3700 4600
NoConn ~ 3800 4600
NoConn ~ 3900 4600
NoConn ~ 4000 4600
NoConn ~ 4300 4600
NoConn ~ 4400 4600
NoConn ~ 4500 4600
NoConn ~ 4600 4600
NoConn ~ 4700 4600
NoConn ~ 4500 3700
NoConn ~ 4400 3700
NoConn ~ 4300 3700
NoConn ~ 4200 3700
NoConn ~ 4100 3700
NoConn ~ 4000 3700
NoConn ~ 3900 3700
NoConn ~ 3800 3700
NoConn ~ 3500 3700
NoConn ~ 5650 4100
NoConn ~ 5950 3150
Wire Wire Line
	4750 2250 6700 2250
Wire Wire Line
	5950 2350 5950 2150
Wire Wire Line
	5950 2150 6500 2150
Wire Wire Line
	6500 2150 6500 3200
Wire Wire Line
	6500 3250 5950 3250
Wire Wire Line
	5950 3250 5950 3300
$Comp
L Interface_UART:MAX485E U4
U 1 1 616E2D0F
P 5750 3700
F 0 "U4" V 5796 3056 50  0000 R CNN
F 1 "MAX485E" V 5705 3056 50  0000 R CNN
F 2 "escolleraHuellas:max485_TTL_RS485" H 5750 3000 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX1487E-MAX491E.pdf" H 5750 3750 50  0001 C CNN
	1    5750 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6600 3450 6600 3100
Wire Wire Line
	6600 2050 5650 2050
Wire Wire Line
	5650 2050 5650 2350
Wire Wire Line
	6750 3100 6600 3100
Connection ~ 6600 3100
Wire Wire Line
	6600 3100 6600 2050
Wire Wire Line
	6750 3200 6500 3200
Connection ~ 6500 3200
Wire Wire Line
	6500 3200 6500 3250
Wire Wire Line
	6750 3850 6500 3850
Wire Wire Line
	6500 3850 6500 3250
Connection ~ 6500 3250
Wire Wire Line
	6600 3450 6600 3750
Wire Wire Line
	6600 3750 6750 3750
Connection ~ 6600 3450
Wire Wire Line
	6750 3950 6400 3950
Wire Wire Line
	5650 3300 5800 3300
Wire Wire Line
	5800 3300 5800 3450
Wire Wire Line
	5800 3450 6600 3450
$EndSCHEMATC
