EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
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
L MCU_ST_STM32F4:STM32F469VETx U?
U 1 1 6088A7AE
P 6200 3950
AR Path="/6088A7AE" Ref="U?"  Part="1" 
AR Path="/60885E1D/6088A7AE" Ref="U6"  Part="1" 
F 0 "U6" H 6150 1161 50  0000 C CNN
F 1 "STM32F469VETx" H 6150 1070 50  0000 C CNN
F 2 "Package_QFP:LQFP-100_14x14mm_P0.5mm" H 5200 1450 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00219980.pdf" H 6200 3950 50  0001 C CNN
	1    6200 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2550 3950 2550
Wire Wire Line
	5000 2650 3950 2650
Wire Wire Line
	5000 2750 3950 2750
Wire Wire Line
	5000 2850 3950 2850
Wire Wire Line
	5000 2950 3950 2950
Wire Wire Line
	5000 3050 3950 3050
Text HLabel 3950 2550 0    50   Output ~ 0
DSI_D0P
Text HLabel 3950 2650 0    50   Output ~ 0
DSI_D0N
Text HLabel 3950 2750 0    50   Output ~ 0
DSI_CKP
Text HLabel 3950 2850 0    50   Output ~ 0
DSI_CKN
Text HLabel 3950 3050 0    50   Output ~ 0
DSI_D1N
$Comp
L Device:C_Small C8
U 1 1 608B7AEA
P 3450 1200
F 0 "C8" H 3542 1246 50  0000 L CNN
F 1 "2.2uF" H 3542 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3450 1200 50  0001 C CNN
F 3 "~" H 3450 1200 50  0001 C CNN
	1    3450 1200
	1    0    0    -1  
$EndComp
Text Label 4000 2050 0    50   ~ 0
VCAP_1
Text Label 4000 2150 0    50   ~ 0
VCAP_2
Wire Wire Line
	4000 2050 5000 2050
Wire Wire Line
	4000 2150 5000 2150
$Comp
L Device:C_Small C6
U 1 1 6090BE92
P 1250 4600
F 0 "C6" H 1342 4646 50  0000 L CNN
F 1 "1uF" H 1342 4555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1250 4600 50  0001 C CNN
F 3 "~" H 1250 4600 50  0001 C CNN
	1    1250 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 4800 1250 4850
Wire Wire Line
	5000 2350 4000 2350
Text Label 4000 2350 0    50   ~ 0
VDDA
$Comp
L Air_Pollution_Sensor-rescue:USB3075-30-A-Air_Pollution_Sensor-cache J2
U 1 1 6092325B
P 10400 2800
F 0 "J2" H 10730 2846 50  0000 L CNN
F 1 "USB3075-30-A" H 10730 2755 50  0000 L CNN
F 2 "Air_Pollution_Sensor:GCT_USB3075-30-A" H 10400 2800 50  0001 L BNN
F 3 "" H 10400 2800 50  0001 L BNN
F 4 "D3" H 10400 2800 50  0001 L BNN "PARTREV"
F 5 "2.66mm" H 10400 2800 50  0001 L BNN "MAXIMUM_PACKAGE_HEIGHT"
F 6 "Manufacturer recommendations" H 10400 2800 50  0001 L BNN "STANDARD"
F 7 "Global Connector Technology" H 10400 2800 50  0001 L BNN "MANUFACTURER"
	1    10400 2800
	1    0    0    -1  
$EndComp
Text Notes 3650 -800 0    50   ~ 10
VCAPDSI pin is the output of DSI Regulator (1.2 V), which must be connected externally to VDD12DSI.
Text Notes 3650 -1050 0    50   ~ 10
VDDDSI is an independent DSI power supply dedicated for DSI Regulator and MIPI \nD-PHY. This supply must be connected to global VDD.
Wire Wire Line
	6100 1350 6100 1200
Wire Wire Line
	6400 1350 6400 1200
Wire Wire Line
	6400 1200 6300 1200
Wire Wire Line
	6300 1350 6300 1200
Connection ~ 6300 1200
Wire Wire Line
	6300 1200 6200 1200
Wire Wire Line
	6200 1350 6200 1200
Connection ~ 6200 1200
Wire Wire Line
	6200 1200 6100 1200
Wire Wire Line
	6100 1200 6000 1200
Connection ~ 6100 1200
Wire Wire Line
	1300 1100 1300 1000
Connection ~ 1300 1400
$Comp
L Device:C_Small C7
U 1 1 609364C6
P 1300 1200
F 0 "C7" H 1392 1246 50  0000 L CNN
F 1 "100nF" H 1392 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1300 1200 50  0001 C CNN
F 3 "~" H 1300 1200 50  0001 C CNN
	1    1300 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 60935348
P 900 1200
F 0 "C3" H 992 1246 50  0000 L CNN
F 1 "4.7uF" H 992 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 900 1200 50  0001 C CNN
F 3 "~" H 900 1200 50  0001 C CNN
	1    900  1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 609605C8
P 1650 4600
F 0 "C9" H 1742 4646 50  0000 L CNN
F 1 "100nF" H 1742 4555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1650 4600 50  0001 C CNN
F 3 "~" H 1650 4600 50  0001 C CNN
	1    1650 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 4500 1650 4450
Wire Wire Line
	1650 4450 1250 4450
Wire Wire Line
	1250 4800 1650 4800
Wire Wire Line
	1250 4700 1250 4800
Connection ~ 1250 4800
Wire Wire Line
	1650 4800 1650 4700
Wire Wire Line
	5000 2250 4000 2250
Text Label 4000 2250 0    50   ~ 0
VCAPDSI
Wire Wire Line
	1650 4800 2100 4800
Connection ~ 1650 4800
Text Label 2100 4800 2    50   ~ 0
VSSA
Wire Wire Line
	1650 4450 2100 4450
Connection ~ 1650 4450
Text Label 2100 4450 2    50   ~ 0
VDDA
Text Label 2300 2400 2    50   ~ 0
VDD12DSI
$Comp
L Device:C_Small C13
U 1 1 60986120
P 1750 2600
F 0 "C13" H 1842 2646 50  0000 L CNN
F 1 "2.2uF" H 1842 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1750 2600 50  0001 C CNN
F 3 "~" H 1750 2600 50  0001 C CNN
	1    1750 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2500 1750 2400
Wire Wire Line
	1750 2400 2300 2400
Wire Wire Line
	1750 2850 2300 2850
Wire Wire Line
	1750 2200 2300 2200
Text Label 2300 2200 2    50   ~ 0
VCAPDSI
Wire Wire Line
	1750 2200 1750 2400
Connection ~ 1750 2400
Connection ~ 1750 2850
Wire Wire Line
	10000 2500 9200 2500
Wire Wire Line
	10000 2600 9200 2600
Wire Wire Line
	10000 2700 9200 2700
$Comp
L Device:C_Small C16
U 1 1 60B0E93E
P 9750 3350
F 0 "C16" H 9842 3396 50  0000 L CNN
F 1 "4.7nF" H 9842 3305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9750 3350 50  0001 C CNN
F 3 "~" H 9750 3350 50  0001 C CNN
	1    9750 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 2900 10000 2900
$Comp
L Device:R_Small_US R3
U 1 1 60B178FB
P 9550 3350
F 0 "R3" H 9618 3396 50  0000 L CNN
F 1 "1M" H 9618 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 9550 3350 50  0001 C CNN
F 3 "~" H 9550 3350 50  0001 C CNN
	1    9550 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 3100 9750 3100
Wire Wire Line
	9550 3100 9550 3250
Wire Wire Line
	9750 3250 9750 3100
Connection ~ 9750 3100
Wire Wire Line
	9750 3100 9550 3100
Wire Wire Line
	9550 3450 9550 3550
Wire Wire Line
	9550 3550 9650 3550
Wire Wire Line
	9750 3550 9750 3450
Wire Wire Line
	9250 2900 9250 3700
Wire Wire Line
	9650 3700 9650 3550
Connection ~ 9650 3550
Wire Wire Line
	9650 3550 9750 3550
Text Label 9250 2500 0    50   ~ 0
USB_OTG_FS_VBUS
Text Label 9250 2600 0    50   ~ 0
USB_DN
Text Label 9250 2700 0    50   ~ 0
USB_DP
Wire Wire Line
	900  2750 900  2950
Wire Wire Line
	7300 2750 8300 2750
Wire Wire Line
	7300 2850 8300 2850
Text Label 8300 2850 2    50   ~ 0
USB_OTG_FS_DP
Text Label 8300 2750 2    50   ~ 0
USB_OTG_FS_DN
Wire Wire Line
	7300 2550 8300 2550
Text Label 8300 2550 2    50   ~ 0
USB_OTG_FS_VBUS
Wire Wire Line
	5000 3850 3950 3850
Text HLabel 3950 3850 0    50   Output ~ 0
RCC_OSC_OUT
Text HLabel 3950 2950 0    50   Output ~ 0
DSI_D1P
Wire Wire Line
	5000 3750 3950 3750
Text HLabel 3950 3750 0    50   Input ~ 0
RCC_OSC_IN
$Comp
L ECMF02-2AMX6:ECMF02-2AMX6 U9
U 1 1 61581622
P 8700 1050
F 0 "U9" H 9500 1437 60  0000 C CNN
F 1 "ECMF02-2AMX6" H 9500 1331 60  0000 C CNN
F 2 "Air_Pollution_Sensor:ECMF02-2AMX6" H 9500 1290 60  0001 C CNN
F 3 "" H 8700 1050 60  0000 C CNN
	1    8700 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1150 7700 1150
Wire Wire Line
	8700 1050 7700 1050
Text Label 7700 1050 0    50   ~ 0
USB_OTG_FS_DP
Text Label 7700 1150 0    50   ~ 0
USB_OTG_FS_DN
Wire Wire Line
	10300 1150 11100 1150
Wire Wire Line
	10300 1050 11100 1050
Text Label 11050 1150 2    50   ~ 0
USB_DN
Text Label 11050 1050 2    50   ~ 0
USB_DP
Wire Wire Line
	7300 2050 8300 2050
Wire Wire Line
	7300 2150 8300 2150
Wire Wire Line
	7300 2250 8300 2250
Wire Wire Line
	7300 2350 8300 2350
Text Label 8300 2150 2    50   ~ 0
SPI_SCK
Text Label 8300 2250 2    50   ~ 0
SPI_MISO
Text Label 8300 2350 2    50   ~ 0
SPI_MOSI
Text Label 8300 2050 2    50   ~ 0
SPI_CS_N
Text HLabel 8300 2050 2    50   Output ~ 0
SPI_CS_N
Text HLabel 8300 2150 2    50   Output ~ 0
SPI_SCK
Text HLabel 8300 2250 2    50   Input ~ 0
SPI_MISO
Text HLabel 8300 2350 2    50   Output ~ 0
SPI_MOSI
$Comp
L Air_Pollution_Sensor-cache:+3.3V #PWR0148
U 1 1 61626600
P 900 850
F 0 "#PWR0148" H 900 700 50  0001 C CNN
F 1 "+3.3V" H 915 1023 50  0000 C CNN
F 2 "" H 900 850 50  0001 C CNN
F 3 "" H 900 850 50  0001 C CNN
	1    900  850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 609CA639
P 900 2650
F 0 "C4" H 992 2696 50  0000 L CNN
F 1 "100nF" H 992 2605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 900 2650 50  0001 C CNN
F 3 "~" H 900 2650 50  0001 C CNN
	1    900  2650
	1    0    0    -1  
$EndComp
$Comp
L Air_Pollution_Sensor-cache:+3.3V #PWR0149
U 1 1 616D5EE5
P 900 2050
F 0 "#PWR0149" H 900 1900 50  0001 C CNN
F 1 "+3.3V" H 915 2223 50  0000 C CNN
F 2 "" H 900 2050 50  0001 C CNN
F 3 "" H 900 2050 50  0001 C CNN
	1    900  2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 4150 1250 4450
Connection ~ 1250 4450
Wire Wire Line
	1250 4450 1250 4500
$Comp
L Air_Pollution_Sensor-cache:+3.3V #PWR0163
U 1 1 60B14DFB
P 1250 4150
F 0 "#PWR0163" H 1250 4000 50  0001 C CNN
F 1 "+3.3V" H 1265 4323 50  0000 C CNN
F 2 "" H 1250 4150 50  0001 C CNN
F 3 "" H 1250 4150 50  0001 C CNN
	1    1250 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 1200 6000 1350
Wire Wire Line
	5900 1350 5900 800 
Wire Wire Line
	6000 1200 6000 800 
Connection ~ 6000 1200
$Comp
L Air_Pollution_Sensor-cache:+3.3V #PWR0164
U 1 1 60B2DA68
P 5900 800
F 0 "#PWR0164" H 5900 650 50  0001 C CNN
F 1 "+3.3V" H 5915 973 50  0000 C CNN
F 2 "" H 5900 800 50  0001 C CNN
F 3 "" H 5900 800 50  0001 C CNN
	1    5900 800 
	1    0    0    -1  
$EndComp
$Comp
L Air_Pollution_Sensor-cache:+3.3V #PWR0165
U 1 1 60B3199D
P 6000 800
F 0 "#PWR0165" H 6000 650 50  0001 C CNN
F 1 "+3.3V" H 6015 973 50  0000 C CNN
F 2 "" H 6000 800 50  0001 C CNN
F 3 "" H 6000 800 50  0001 C CNN
	1    6000 800 
	1    0    0    -1  
$EndComp
$Comp
L Air_Pollution_Sensor-cache:+3.3V #PWR0166
U 1 1 60B36174
P 6700 750
F 0 "#PWR0166" H 6700 600 50  0001 C CNN
F 1 "+3.3V" H 6715 923 50  0000 C CNN
F 2 "" H 6700 750 50  0001 C CNN
F 3 "" H 6700 750 50  0001 C CNN
	1    6700 750 
	1    0    0    -1  
$EndComp
$Comp
L Air_Pollution_Sensor-cache:+3.3V #PWR0167
U 1 1 60B39E67
P 6800 750
F 0 "#PWR0167" H 6800 600 50  0001 C CNN
F 1 "+3.3V" H 6815 923 50  0000 C CNN
F 2 "" H 6800 750 50  0001 C CNN
F 3 "" H 6800 750 50  0001 C CNN
	1    6800 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 750  6800 1350
Wire Wire Line
	7300 2950 8300 2950
Wire Wire Line
	7300 3050 8300 3050
Text Label 8300 3050 2    50   ~ 0
swclk
Text Label 8300 2950 2    50   ~ 0
swdio
Text HLabel 8300 2950 2    50   Input ~ 0
swdio
Text HLabel 8300 3050 2    50   Input ~ 0
swclk
Text HLabel 3950 5150 0    50   Input ~ 0
LCD_RESET
Wire Wire Line
	5000 5150 3950 5150
Wire Wire Line
	900  850  900  1000
Wire Wire Line
	900  1000 1300 1000
Connection ~ 900  1000
Wire Wire Line
	900  1000 900  1100
Wire Wire Line
	900  1400 1300 1400
Connection ~ 900  1400
Wire Wire Line
	1300 1400 1700 1400
Connection ~ 1700 1400
Wire Wire Line
	1700 1400 2200 1400
Connection ~ 2200 1400
Wire Wire Line
	900  1300 900  1400
Wire Wire Line
	1300 1300 1300 1400
Wire Wire Line
	1700 1300 1700 1400
Wire Wire Line
	2200 1300 2200 1400
Wire Wire Line
	8700 1250 8700 1500
Text Label 3450 700  3    50   ~ 0
VCAP_2
Wire Wire Line
	3450 1100 3450 700 
$Comp
L Device:C_Small C5
U 1 1 608BCE93
P 3050 1200
F 0 "C5" H 3142 1246 50  0000 L CNN
F 1 "2.2uF" H 3142 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3050 1200 50  0001 C CNN
F 3 "~" H 3050 1200 50  0001 C CNN
	1    3050 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 1400 3050 1300
Wire Wire Line
	3050 1400 3450 1400
Wire Wire Line
	3450 1400 3450 1300
Connection ~ 3050 1400
Text Label 3050 700  3    50   ~ 0
VCAP_1
Wire Wire Line
	3050 1100 3050 700 
Text Label 900  2200 3    50   ~ 0
VDDUSB
Wire Wire Line
	900  2050 900  2550
Text Label 6800 950  3    50   ~ 0
VDDUSB
Text Label 2300 2850 2    50   ~ 0
VSSDSI
Wire Wire Line
	6000 7300 6000 6650
Text Label 3800 1400 2    50   ~ 0
VSS
Connection ~ 3450 1400
Wire Wire Line
	3450 1400 3800 1400
Wire Wire Line
	900  1400 900  1550
Wire Wire Line
	1750 2850 1750 3100
Wire Wire Line
	1750 2700 1750 2850
$Comp
L power:GND #PWR0133
U 1 1 61578C71
P 900 2950
F 0 "#PWR0133" H 900 2700 50  0001 C CNN
F 1 "GND" H 905 2777 50  0000 C CNN
F 2 "" H 900 2950 50  0001 C CNN
F 3 "" H 900 2950 50  0001 C CNN
	1    900  2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0142
U 1 1 6157C1D1
P 1250 4850
F 0 "#PWR0142" H 1250 4600 50  0001 C CNN
F 1 "GND" H 1255 4677 50  0000 C CNN
F 2 "" H 1250 4850 50  0001 C CNN
F 3 "" H 1250 4850 50  0001 C CNN
	1    1250 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0161
U 1 1 6157F772
P 8700 1500
F 0 "#PWR0161" H 8700 1250 50  0001 C CNN
F 1 "GND" H 8705 1327 50  0000 C CNN
F 2 "" H 8700 1500 50  0001 C CNN
F 3 "" H 8700 1500 50  0001 C CNN
	1    8700 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0162
U 1 1 61582C6C
P 9650 3700
F 0 "#PWR0162" H 9650 3450 50  0001 C CNN
F 1 "GND" H 9655 3527 50  0000 C CNN
F 2 "" H 9650 3700 50  0001 C CNN
F 3 "" H 9650 3700 50  0001 C CNN
	1    9650 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0168
U 1 1 6158605C
P 9250 3700
F 0 "#PWR0168" H 9250 3450 50  0001 C CNN
F 1 "GND" H 9255 3527 50  0000 C CNN
F 2 "" H 9250 3700 50  0001 C CNN
F 3 "" H 9250 3700 50  0001 C CNN
	1    9250 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 6650 6400 6650
Connection ~ 6000 6650
Connection ~ 6100 6650
Wire Wire Line
	6100 6650 6000 6650
Connection ~ 6200 6650
Wire Wire Line
	6200 6650 6100 6650
Connection ~ 6300 6650
Wire Wire Line
	6300 6650 6200 6650
Connection ~ 6400 6650
Wire Wire Line
	6400 6650 6300 6650
$Comp
L power:GND #PWR0172
U 1 1 6158D7A3
P 6000 7300
F 0 "#PWR0172" H 6000 7050 50  0001 C CNN
F 1 "GND" H 6005 7127 50  0000 C CNN
F 2 "" H 6000 7300 50  0001 C CNN
F 3 "" H 6000 7300 50  0001 C CNN
	1    6000 7300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0180
U 1 1 61594A4A
P 900 1550
F 0 "#PWR0180" H 900 1300 50  0001 C CNN
F 1 "GND" H 905 1377 50  0000 C CNN
F 2 "" H 900 1550 50  0001 C CNN
F 3 "" H 900 1550 50  0001 C CNN
	1    900  1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 6650 6600 7300
Text Label 6600 7150 1    50   ~ 0
VSSDSI
$Comp
L Device:Ferrite_Bead FB1
U 1 1 6159DBB8
P 1750 5800
F 0 "FB1" V 1476 5800 50  0000 C CNN
F 1 "Ferrite_Bead" V 1567 5800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" V 1680 5800 50  0001 C CNN
F 3 "~" H 1750 5800 50  0001 C CNN
	1    1750 5800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0181
U 1 1 615A75D9
P 1300 5900
F 0 "#PWR0181" H 1300 5650 50  0001 C CNN
F 1 "GND" H 1305 5727 50  0000 C CNN
F 2 "" H 1300 5900 50  0001 C CNN
F 3 "" H 1300 5900 50  0001 C CNN
	1    1300 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR0182
U 1 1 615AB164
P 2050 5900
F 0 "#PWR0182" H 2050 5650 50  0001 C CNN
F 1 "GND1" H 2055 5727 50  0000 C CNN
F 2 "" H 2050 5900 50  0001 C CNN
F 3 "" H 2050 5900 50  0001 C CNN
	1    2050 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR0183
U 1 1 615ACABC
P 6600 7300
F 0 "#PWR0183" H 6600 7050 50  0001 C CNN
F 1 "GND1" H 6605 7127 50  0000 C CNN
F 2 "" H 6600 7300 50  0001 C CNN
F 3 "" H 6600 7300 50  0001 C CNN
	1    6600 7300
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR0184
U 1 1 615B0860
P 1750 3100
F 0 "#PWR0184" H 1750 2850 50  0001 C CNN
F 1 "GND1" H 1755 2927 50  0000 C CNN
F 2 "" H 1750 3100 50  0001 C CNN
F 3 "" H 1750 3100 50  0001 C CNN
	1    1750 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 5900 1300 5800
Wire Wire Line
	1300 5800 1600 5800
Wire Wire Line
	1900 5800 2050 5800
Wire Wire Line
	2050 5800 2050 5900
$Comp
L Device:C_Small C12
U 1 1 60938A66
P 4750 850
F 0 "C12" H 4842 896 50  0000 L CNN
F 1 "100nF" H 4842 805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 4750 850 50  0001 C CNN
F 3 "~" H 4750 850 50  0001 C CNN
	1    4750 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 750  6700 850 
Text Label 6700 1250 1    50   ~ 0
VDDDSI
Wire Wire Line
	6600 850  6700 850 
Wire Wire Line
	6600 850  6600 1350
Connection ~ 6700 850 
Wire Wire Line
	6700 850  6700 1350
Text Label 4650 750  2    50   ~ 0
VDDDSI
Wire Wire Line
	4250 750  4750 750 
$Comp
L power:GND1 #PWR0190
U 1 1 61604031
P 4750 1050
F 0 "#PWR0190" H 4750 800 50  0001 C CNN
F 1 "GND1" H 4755 877 50  0000 C CNN
F 2 "" H 4750 1050 50  0001 C CNN
F 3 "" H 4750 1050 50  0001 C CNN
	1    4750 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 950  4750 1050
Wire Wire Line
	2200 1400 2600 1400
Connection ~ 1300 1000
Wire Wire Line
	1700 1000 2200 1000
Wire Wire Line
	2200 1000 2200 1100
Connection ~ 1700 1000
Wire Wire Line
	1700 1000 1700 1100
Wire Wire Line
	1300 1000 1700 1000
$Comp
L Device:C_Small C11
U 1 1 60937C9D
P 2200 1200
F 0 "C11" H 2292 1246 50  0000 L CNN
F 1 "100nF" H 2292 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2200 1200 50  0001 C CNN
F 3 "~" H 2200 1200 50  0001 C CNN
	1    2200 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 6093714D
P 1700 1200
F 0 "C10" H 1792 1246 50  0000 L CNN
F 1 "100nF" H 1792 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1700 1200 50  0001 C CNN
F 3 "~" H 1700 1200 50  0001 C CNN
	1    1700 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 607C977A
P 4550 5450
AR Path="/6103AB31/607C977A" Ref="R?"  Part="1" 
AR Path="/60885E1D/607C977A" Ref="R5"  Part="1" 
F 0 "R5" V 4345 5450 50  0000 C CNN
F 1 "1k" V 4436 5450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4550 5450 50  0001 C CNN
F 3 "~" H 4550 5450 50  0001 C CNN
	1    4550 5450
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D?
U 1 1 607C9780
P 4250 5850
AR Path="/6103AB31/607C9780" Ref="D?"  Part="1" 
AR Path="/60885E1D/607C9780" Ref="D5"  Part="1" 
F 0 "D5" V 4289 5732 50  0000 R CNN
F 1 "LED" V 4198 5732 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4250 5850 50  0001 C CNN
F 3 "~" H 4250 5850 50  0001 C CNN
	1    4250 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 607C9787
P 4250 6250
AR Path="/6103AB31/607C9787" Ref="#PWR?"  Part="1" 
AR Path="/60885E1D/607C9787" Ref="#PWR0156"  Part="1" 
F 0 "#PWR0156" H 4250 6000 50  0001 C CNN
F 1 "GND" H 4255 6077 50  0000 C CNN
F 2 "" H 4250 6250 50  0001 C CNN
F 3 "" H 4250 6250 50  0001 C CNN
	1    4250 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 6000 4250 6250
Wire Wire Line
	5000 5450 4650 5450
Wire Wire Line
	4450 5450 4250 5450
Wire Wire Line
	4250 5450 4250 5700
$Comp
L Connector:TestPoint TP?
U 1 1 60A5F2B0
P 5000 6900
AR Path="/6093D452/60A5F2B0" Ref="TP?"  Part="1" 
AR Path="/60885E1D/60A5F2B0" Ref="TP12"  Part="1" 
F 0 "TP12" H 5058 7018 50  0000 L CNN
F 1 "TestPoint" H 5058 6927 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 5200 6900 50  0001 C CNN
F 3 "~" H 5200 6900 50  0001 C CNN
	1    5000 6900
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP?
U 1 1 60A63331
P 4750 6900
AR Path="/6093D452/60A63331" Ref="TP?"  Part="1" 
AR Path="/60885E1D/60A63331" Ref="TP11"  Part="1" 
F 0 "TP11" H 4808 7018 50  0000 L CNN
F 1 "TestPoint" H 4808 6927 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 4950 6900 50  0001 C CNN
F 3 "~" H 4950 6900 50  0001 C CNN
	1    4750 6900
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP?
U 1 1 60A672BA
P 4550 6900
AR Path="/6093D452/60A672BA" Ref="TP?"  Part="1" 
AR Path="/60885E1D/60A672BA" Ref="TP10"  Part="1" 
F 0 "TP10" H 4608 7018 50  0000 L CNN
F 1 "TestPoint" H 4608 6927 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 4750 6900 50  0001 C CNN
F 3 "~" H 4750 6900 50  0001 C CNN
	1    4550 6900
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP?
U 1 1 60A6B139
P 4400 6900
AR Path="/6093D452/60A6B139" Ref="TP?"  Part="1" 
AR Path="/60885E1D/60A6B139" Ref="TP9"  Part="1" 
F 0 "TP9" H 4458 7018 50  0000 L CNN
F 1 "TestPoint" H 4458 6927 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 4600 6900 50  0001 C CNN
F 3 "~" H 4600 6900 50  0001 C CNN
	1    4400 6900
	-1   0    0    1   
$EndComp
Wire Wire Line
	5000 6050 4400 6050
Wire Wire Line
	4400 6050 4400 6900
Wire Wire Line
	4550 6900 4550 6150
Wire Wire Line
	4550 6150 5000 6150
Wire Wire Line
	4750 6900 4750 6250
Wire Wire Line
	4750 6250 5000 6250
Wire Wire Line
	5000 6900 5000 6350
Text Label 6500 900  3    50   ~ 0
VDD12DSI
Text Label 6600 1050 3    50   ~ 0
VDDA
$Comp
L Device:C_Small C34
U 1 1 60BA1574
P 2600 1200
F 0 "C34" H 2692 1246 50  0000 L CNN
F 1 "100nF" H 2692 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2600 1200 50  0001 C CNN
F 3 "~" H 2600 1200 50  0001 C CNN
	1    2600 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1000 2600 1000
Wire Wire Line
	2600 1000 2600 1100
Connection ~ 2200 1000
Wire Wire Line
	2600 1300 2600 1400
Connection ~ 2600 1400
Wire Wire Line
	2600 1400 3050 1400
Wire Wire Line
	6500 850  6500 1350
Text Notes 7400 7500 0    50   ~ 10
MCU_STM32F469_module
Text Notes 10600 7650 0    50   ~ 10
1.0
Text Notes 8150 7650 0    50   ~ 10
2021. 04. 13
$EndSCHEMATC
