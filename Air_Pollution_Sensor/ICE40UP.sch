EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 4
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
L FPGA_Lattice:ICE40UP5K-SG48ITR U?
U 1 1 61070DEB
P 2300 3550
AR Path="/61070DEB" Ref="U?"  Part="1" 
AR Path="/6093CD3D/61070DEB" Ref="U?"  Part="1" 
AR Path="/6103AB31/61070DEB" Ref="U2"  Part="1" 
F 0 "U2" H 2630 3603 50  0000 L CNN
F 1 "ICE40UP5K-SG48ITR" H 2630 3512 50  0000 L CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm" H 2300 2200 50  0001 C CNN
F 3 "http://www.latticesemi.com/Products/FPGAandCPLD/iCE40Ultra" H 1900 4550 50  0001 C CNN
	1    2300 3550
	1    0    0    -1  
$EndComp
Text HLabel 10000 5450 0    50   Input ~ 0
SCL
Text HLabel 10000 5550 0    50   3State ~ 0
SDA
Connection ~ 2300 5950
Wire Wire Line
	2300 5950 2300 6150
$Comp
L FPGA_Lattice:ICE40UP5K-SG48ITR U?
U 3 1 61070E3F
P 8150 3300
AR Path="/61070E3F" Ref="U?"  Part="3" 
AR Path="/6093CD3D/61070E3F" Ref="U?"  Part="3" 
AR Path="/6103AB31/61070E3F" Ref="U2"  Part="3" 
F 0 "U2" H 8480 3403 50  0000 L CNN
F 1 "ICE40UP5K-SG48ITR" H 8480 3312 50  0000 L CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm" H 8150 1950 50  0001 C CNN
F 3 "http://www.latticesemi.com/Products/FPGAandCPLD/iCE40Ultra" H 7750 4300 50  0001 C CNN
	3    8150 3300
	1    0    0    -1  
$EndComp
$Comp
L FPGA_Lattice:ICE40UP5K-SG48ITR U?
U 2 1 61070F1E
P 5050 3800
AR Path="/61070F1E" Ref="U?"  Part="2" 
AR Path="/6093CD3D/61070F1E" Ref="U?"  Part="2" 
AR Path="/6103AB31/61070F1E" Ref="U2"  Part="2" 
F 0 "U2" H 5050 2725 50  0000 C CNN
F 1 "ICE40UP5K-SG48ITR" H 5050 2634 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm" H 5050 2450 50  0001 C CNN
F 3 "http://www.latticesemi.com/Products/FPGAandCPLD/iCE40Ultra" H 4650 4800 50  0001 C CNN
	2    5050 3800
	1    0    0    -1  
$EndComp
Text Label 3400 4600 0    50   ~ 0
SF_SPI_SCK
Text Label 3400 4700 0    50   ~ 0
SF_SPI_SS
Text Label 3900 3100 0    50   ~ 0
CRESET_B
$Comp
L Device:C_Small C?
U 1 1 61070F9C
P 9250 1600
AR Path="/6093CD3D/61070F9C" Ref="C?"  Part="1" 
AR Path="/6103AB31/61070F9C" Ref="C17"  Part="1" 
F 0 "C17" H 9342 1646 50  0000 L CNN
F 1 "1uF" H 9342 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9250 1600 50  0001 C CNN
F 3 "~" H 9250 1600 50  0001 C CNN
	1    9250 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 61070FA2
P 9750 1600
AR Path="/6093CD3D/61070FA2" Ref="C?"  Part="1" 
AR Path="/6103AB31/61070FA2" Ref="C19"  Part="1" 
F 0 "C19" H 9842 1646 50  0000 L CNN
F 1 "100nF" H 9842 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9750 1600 50  0001 C CNN
F 3 "~" H 9750 1600 50  0001 C CNN
	1    9750 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 1700 9250 1850
Wire Wire Line
	9250 1850 9750 1850
Wire Wire Line
	9750 1700 9750 1850
Wire Wire Line
	9250 1500 9250 1350
Wire Wire Line
	9250 1350 9750 1350
Wire Wire Line
	9750 1500 9750 1350
Connection ~ 9750 1350
Wire Wire Line
	1900 3850 1200 3850
Wire Wire Line
	1900 3250 1200 3250
Wire Wire Line
	1900 3150 1200 3150
Wire Wire Line
	1900 3050 1200 3050
Wire Wire Line
	1900 2950 1200 2950
Text Label 1200 3850 0    50   ~ 0
ICE_CLK
Text HLabel 1200 3050 0    50   Output ~ 0
spi_mosi
Text HLabel 1200 3150 0    50   Input ~ 0
spi_miso
Text HLabel 1200 3250 0    50   Output ~ 0
spi_sck
Text HLabel 1200 2950 0    50   Output ~ 0
spi_cs
Wire Wire Line
	9750 1350 10050 1350
Text Label 10700 5450 2    50   ~ 0
SCL
Text Label 10700 5550 2    50   ~ 0
SDA
Connection ~ 1000 6850
Wire Wire Line
	1000 6850 1000 6700
Connection ~ 2250 6850
Wire Wire Line
	2250 6700 2250 6850
$Comp
L power:+1V2 #PWR?
U 1 1 61071119
P 2250 6700
AR Path="/6093CD3D/61071119" Ref="#PWR?"  Part="1" 
AR Path="/6103AB31/61071119" Ref="#PWR0135"  Part="1" 
F 0 "#PWR0135" H 2250 6550 50  0001 C CNN
F 1 "+1V2" H 2265 6873 50  0000 C CNN
F 2 "" H 2250 6700 50  0001 C CNN
F 3 "" H 2250 6700 50  0001 C CNN
	1    2250 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 6850 1350 6850
Wire Wire Line
	1000 6900 1000 6850
Wire Wire Line
	1000 7250 1000 7100
Wire Wire Line
	1650 7150 1650 7250
Wire Wire Line
	2250 6850 1950 6850
Wire Wire Line
	2250 6950 2250 6850
Wire Wire Line
	2250 7250 2250 7150
$Comp
L Device:C_Small C?
U 1 1 610710F0
P 2250 7050
AR Path="/6093CD3D/610710F0" Ref="C?"  Part="1" 
AR Path="/6103AB31/610710F0" Ref="C43"  Part="1" 
F 0 "C43" H 2342 7096 50  0000 L CNN
F 1 "10uF" H 2342 7005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2250 7050 50  0001 C CNN
F 3 "~" H 2250 7050 50  0001 C CNN
	1    2250 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 610710EA
P 1000 7000
AR Path="/6093CD3D/610710EA" Ref="C?"  Part="1" 
AR Path="/6103AB31/610710EA" Ref="C42"  Part="1" 
F 0 "C42" H 1092 7046 50  0000 L CNN
F 1 "10uF" H 1092 6955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1000 7000 50  0001 C CNN
F 3 "~" H 1000 7000 50  0001 C CNN
	1    1000 7000
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LD1117S12TR_SOT223 U?
U 1 1 610710E4
P 1650 6850
AR Path="/6093CD3D/610710E4" Ref="U?"  Part="1" 
AR Path="/6103AB31/610710E4" Ref="U8"  Part="1" 
F 0 "U8" H 1650 7092 50  0000 C CNN
F 1 "LD1117S12TR_SOT223" H 1650 7001 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 1650 7050 50  0001 C CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00000544.pdf" H 1750 6600 50  0001 C CNN
	1    1650 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 1000 10050 1350
$Comp
L power:+3.3V #PWR0108
U 1 1 6117862D
P 10050 1000
F 0 "#PWR0108" H 10050 850 50  0001 C CNN
F 1 "+3.3V" H 10065 1173 50  0000 C CNN
F 2 "" H 10050 1000 50  0001 C CNN
F 3 "" H 10050 1000 50  0001 C CNN
	1    10050 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 5600 8950 5250
Wire Wire Line
	8950 5600 8950 5750
Wire Wire Line
	8450 5750 8450 5600
Wire Wire Line
	8450 6100 8950 6100
Wire Wire Line
	8450 5950 8450 6100
Wire Wire Line
	8950 6100 8950 5950
$Comp
L Device:C_Small C?
U 1 1 61071067
P 8950 5850
AR Path="/6093CD3D/61071067" Ref="C?"  Part="1" 
AR Path="/6103AB31/61071067" Ref="C41"  Part="1" 
F 0 "C41" H 9042 5896 50  0000 L CNN
F 1 "10uF" H 9042 5805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8950 5850 50  0001 C CNN
F 3 "~" H 8950 5850 50  0001 C CNN
	1    8950 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 61071061
P 8450 5850
AR Path="/6093CD3D/61071061" Ref="C?"  Part="1" 
AR Path="/6103AB31/61071061" Ref="C39"  Part="1" 
F 0 "C39" H 8542 5896 50  0000 L CNN
F 1 "100nF" H 8542 5805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8450 5850 50  0001 C CNN
F 3 "~" H 8450 5850 50  0001 C CNN
	1    8450 5850
	1    0    0    -1  
$EndComp
$Comp
L FPGA_Lattice:ICE40UP5K-SG48ITR U?
U 4 1 61070E39
P 7550 5400
AR Path="/61070E39" Ref="U?"  Part="4" 
AR Path="/6093CD3D/61070E39" Ref="U?"  Part="4" 
AR Path="/6103AB31/61070E39" Ref="U2"  Part="4" 
F 0 "U2" H 7780 5446 50  0000 L CNN
F 1 "ICE40UP5K-SG48ITR" H 7780 5355 50  0000 L CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm" H 7550 4050 50  0001 C CNN
F 3 "http://www.latticesemi.com/Products/FPGAandCPLD/iCE40Ultra" H 7150 6400 50  0001 C CNN
	4    7550 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 5900 7550 5800
$Comp
L power:+1V2 #PWR0138
U 1 1 613D198F
P 8950 5250
F 0 "#PWR0138" H 8950 5100 50  0001 C CNN
F 1 "+1V2" H 8965 5423 50  0000 C CNN
F 2 "" H 8950 5250 50  0001 C CNN
F 3 "" H 8950 5250 50  0001 C CNN
	1    8950 5250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0141
U 1 1 614513B2
P 1000 6700
F 0 "#PWR0141" H 1000 6550 50  0001 C CNN
F 1 "+3.3V" H 1015 6873 50  0000 C CNN
F 2 "" H 1000 6700 50  0001 C CNN
F 3 "" H 1000 6700 50  0001 C CNN
	1    1000 6700
	1    0    0    -1  
$EndComp
Text HLabel 10000 5650 0    50   Input ~ 0
temp_alert
Text Label 10700 5650 2    50   ~ 0
temp_alert
Text Label 3900 3400 0    50   ~ 0
temp_alert
Wire Wire Line
	3850 3400 4550 3400
Wire Wire Line
	7750 3100 7050 3100
Wire Wire Line
	7750 3200 7050 3200
Text Label 7050 3100 0    50   ~ 0
SDA
Text Label 7050 3200 0    50   ~ 0
SCL
Text HLabel 10000 5850 0    50   Input ~ 0
jtag_tck
Text HLabel 10000 5950 0    50   Input ~ 0
jtag_tdo
Text HLabel 10000 6050 0    50   Input ~ 0
jtag_tdi
Text HLabel 10000 6150 0    50   Input ~ 0
jtag_tms
Wire Wire Line
	1900 3750 1200 3750
Wire Wire Line
	1900 3950 1200 3950
Wire Wire Line
	1900 4150 1200 4150
Wire Wire Line
	1900 4250 1200 4250
Text Label 1200 3750 0    50   ~ 0
jtag_tck
Text Label 1200 3950 0    50   ~ 0
jtag_tdo
Text Label 1200 4150 0    50   ~ 0
jtag_tdi
Text Label 1200 4250 0    50   ~ 0
jtag_tms
Wire Wire Line
	10000 6150 10700 6150
Wire Wire Line
	10000 6050 10700 6050
Wire Wire Line
	10000 5950 10700 5950
Wire Wire Line
	10000 5850 10700 5850
Text Label 10700 6150 2    50   ~ 0
jtag_tms
Text Label 10700 6050 2    50   ~ 0
jtag_tdi
Text Label 10700 5950 2    50   ~ 0
jtag_tdo
Text Label 10700 5850 2    50   ~ 0
jtag_tck
Text Label 3400 4500 0    50   ~ 0
SF_SPI_DI
Wire Wire Line
	3400 4500 4100 4500
Text Label 3400 4400 0    50   ~ 0
SF_SPI_DO
Wire Wire Line
	3400 4400 4100 4400
$Comp
L power:GND #PWR0130
U 1 1 609FCB57
P 1000 7250
F 0 "#PWR0130" H 1000 7000 50  0001 C CNN
F 1 "GND" H 1005 7077 50  0000 C CNN
F 2 "" H 1000 7250 50  0001 C CNN
F 3 "" H 1000 7250 50  0001 C CNN
	1    1000 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 60A03F0C
P 1650 7250
F 0 "#PWR0131" H 1650 7000 50  0001 C CNN
F 1 "GND" H 1655 7077 50  0000 C CNN
F 2 "" H 1650 7250 50  0001 C CNN
F 3 "" H 1650 7250 50  0001 C CNN
	1    1650 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 60A0B332
P 2250 7250
F 0 "#PWR0132" H 2250 7000 50  0001 C CNN
F 1 "GND" H 2255 7077 50  0000 C CNN
F 2 "" H 2250 7250 50  0001 C CNN
F 3 "" H 2250 7250 50  0001 C CNN
	1    2250 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0134
U 1 1 60A28B1A
P 7550 5900
F 0 "#PWR0134" H 7550 5650 50  0001 C CNN
F 1 "GND" H 7555 5727 50  0000 C CNN
F 2 "" H 7550 5900 50  0001 C CNN
F 3 "" H 7550 5900 50  0001 C CNN
	1    7550 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0136
U 1 1 60A3E7D6
P 2300 6150
F 0 "#PWR0136" H 2300 5900 50  0001 C CNN
F 1 "GND" H 2305 5977 50  0000 C CNN
F 2 "" H 2300 6150 50  0001 C CNN
F 3 "" H 2300 6150 50  0001 C CNN
	1    2300 6150
	1    0    0    -1  
$EndComp
Text Label 800  5950 0    50   ~ 0
LED_BLUE
Text Label 800  5550 0    50   ~ 0
LED_GREEN
Text Label 800  5150 0    50   ~ 0
LED_RED
Text Notes 1100 6200 0    50   ~ 0
20mA R2.1V G3.2V B3.2V
Wire Wire Line
	1950 5950 2300 5950
Wire Wire Line
	1950 5550 2300 5550
Wire Wire Line
	1950 5150 2300 5150
Wire Wire Line
	1200 5950 800  5950
Wire Wire Line
	800  5150 1200 5150
Wire Wire Line
	1200 5550 800  5550
Wire Wire Line
	1650 5950 1400 5950
Wire Wire Line
	1650 5550 1400 5550
Wire Wire Line
	1650 5150 1400 5150
$Comp
L Device:LED D?
U 1 1 61070E04
P 1800 5950
AR Path="/61070E04" Ref="D?"  Part="1" 
AR Path="/6093CD3D/61070E04" Ref="D?"  Part="1" 
AR Path="/6103AB31/61070E04" Ref="D3"  Part="1" 
F 0 "D3" H 1793 5695 50  0000 C CNN
F 1 "LED" H 1793 5786 50  0000 C CNN
F 2 "Air_Pollution_Sensor:C503B-GAN-CB0F0791" H 1800 5950 50  0001 C CNN
F 3 "~" H 1800 5950 50  0001 C CNN
	1    1800 5950
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 61070DFE
P 1800 5550
AR Path="/61070DFE" Ref="D?"  Part="1" 
AR Path="/6093CD3D/61070DFE" Ref="D?"  Part="1" 
AR Path="/6103AB31/61070DFE" Ref="D2"  Part="1" 
F 0 "D2" H 1793 5295 50  0000 C CNN
F 1 "LED" H 1793 5386 50  0000 C CNN
F 2 "Air_Pollution_Sensor:C503B-GAN-CB0F0791" H 1800 5550 50  0001 C CNN
F 3 "~" H 1800 5550 50  0001 C CNN
	1    1800 5550
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 61070DF8
P 1800 5150
AR Path="/61070DF8" Ref="D?"  Part="1" 
AR Path="/6093CD3D/61070DF8" Ref="D?"  Part="1" 
AR Path="/6103AB31/61070DF8" Ref="D1"  Part="1" 
F 0 "D1" H 1793 4895 50  0000 C CNN
F 1 "LED" H 1793 4986 50  0000 C CNN
F 2 "Air_Pollution_Sensor:C503B-GAN-CB0F0791" H 1800 5150 50  0001 C CNN
F 3 "~" H 1800 5150 50  0001 C CNN
	1    1800 5150
	-1   0    0    1   
$EndComp
Connection ~ 2300 5550
Wire Wire Line
	2300 5550 2300 5950
Wire Wire Line
	2300 5150 2300 5550
Text Label 1200 4550 0    50   ~ 0
LED_BLUE
Text Label 1200 4450 0    50   ~ 0
LED_GREEN
Text Label 1200 4350 0    50   ~ 0
LED_RED
Wire Wire Line
	1200 4350 1900 4350
Wire Wire Line
	1200 4450 1900 4450
Wire Wire Line
	1200 4550 1900 4550
Wire Wire Line
	4400 6900 4400 7150
Wire Wire Line
	3900 6600 3050 6600
Connection ~ 4400 5600
Wire Wire Line
	3700 6050 3700 6400
Text Label 3050 6400 0    50   ~ 0
SF_SPI_SS
Text Label 3050 6600 0    50   ~ 0
SF_SPI_SCK
Wire Wire Line
	3700 7150 3700 6950
Wire Wire Line
	3700 6400 3700 6750
$Comp
L Device:C_Small C?
U 1 1 60816508
P 5050 6850
AR Path="/6093CD3D/60816508" Ref="C?"  Part="1" 
AR Path="/6103AB31/60816508" Ref="C22"  Part="1" 
F 0 "C22" H 5142 6896 50  0000 L CNN
F 1 "100nF" H 5142 6805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5050 6850 50  0001 C CNN
F 3 "~" H 5050 6850 50  0001 C CNN
	1    5050 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 7150 5050 6950
Wire Wire Line
	3700 5850 3700 5600
Wire Wire Line
	4400 5600 4400 5800
Connection ~ 3700 6400
Wire Wire Line
	3700 6400 3900 6400
Wire Wire Line
	4400 5600 3700 5600
Wire Wire Line
	3050 6400 3700 6400
Wire Wire Line
	5750 5600 5750 5850
Wire Wire Line
	5300 5600 5750 5600
Connection ~ 5300 5600
Wire Wire Line
	5300 5850 5300 5600
Text Label 6250 6400 2    50   ~ 0
SF_SPI_DO
Text Label 6250 6300 2    50   ~ 0
SF_SPI_DI
Text Label 6250 6600 2    50   ~ 0
SST_WP
Wire Wire Line
	5300 6600 6250 6600
Connection ~ 5750 6700
Wire Wire Line
	5750 6700 6250 6700
Text Label 6250 6700 2    50   ~ 0
SST_HOLD
Wire Wire Line
	5750 6050 5750 6700
Connection ~ 5300 6600
Wire Wire Line
	5300 6050 5300 6600
Wire Wire Line
	4900 6300 6250 6300
Wire Wire Line
	4900 6400 6250 6400
Wire Wire Line
	4900 6700 5750 6700
Wire Wire Line
	4900 6600 5300 6600
Wire Wire Line
	4400 5600 5300 5600
Wire Wire Line
	5050 5800 4400 5800
Wire Wire Line
	5050 5800 5050 6750
Connection ~ 4400 5800
Wire Wire Line
	4400 5800 4400 6100
$Comp
L power:+3.3V #PWR0137
U 1 1 6081652C
P 4400 5350
F 0 "#PWR0137" H 4400 5200 50  0001 C CNN
F 1 "+3.3V" H 4415 5523 50  0000 C CNN
F 2 "" H 4400 5350 50  0001 C CNN
F 3 "" H 4400 5350 50  0001 C CNN
	1    4400 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 5350 4400 5600
$Comp
L power:GND #PWR0139
U 1 1 60816533
P 3700 7150
F 0 "#PWR0139" H 3700 6900 50  0001 C CNN
F 1 "GND" H 3705 6977 50  0000 C CNN
F 2 "" H 3700 7150 50  0001 C CNN
F 3 "" H 3700 7150 50  0001 C CNN
	1    3700 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 60816539
P 4400 7150
F 0 "#PWR0140" H 4400 6900 50  0001 C CNN
F 1 "GND" H 4405 6977 50  0000 C CNN
F 2 "" H 4400 7150 50  0001 C CNN
F 3 "" H 4400 7150 50  0001 C CNN
	1    4400 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0143
U 1 1 6081653F
P 5050 7150
F 0 "#PWR0143" H 5050 6900 50  0001 C CNN
F 1 "GND" H 5055 6977 50  0000 C CNN
F 2 "" H 5050 7150 50  0001 C CNN
F 3 "" H 5050 7150 50  0001 C CNN
	1    5050 7150
	1    0    0    -1  
$EndComp
$Comp
L Memory_Flash:W25Q32JVSS U?
U 1 1 60816545
P 4400 6500
AR Path="/6093CD3D/60816545" Ref="U?"  Part="1" 
AR Path="/6103AB31/60816545" Ref="U7"  Part="1" 
F 0 "U7" H 4400 7081 50  0000 C CNN
F 1 "W25Q32JVSS" H 4400 6990 50  0000 C CNN
F 2 "Package_SO:SOIC-8_5.23x5.23mm_P1.27mm" H 4400 6500 50  0001 C CNN
F 3 "http://www.winbond.com/resource-files/w25q32jv%20revg%2003272018%20plus.pdf" H 4400 6500 50  0001 C CNN
	1    4400 6500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R10
U 1 1 6081654B
P 3700 5950
F 0 "R10" H 3768 5996 50  0000 L CNN
F 1 "10K" H 3768 5905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 3700 5950 50  0001 C CNN
F 3 "~" H 3700 5950 50  0001 C CNN
	1    3700 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R11
U 1 1 60816551
P 3700 6850
F 0 "R11" H 3768 6896 50  0000 L CNN
F 1 "1K" H 3768 6805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 3700 6850 50  0001 C CNN
F 3 "~" H 3700 6850 50  0001 C CNN
	1    3700 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R14
U 1 1 60816557
P 5300 5950
F 0 "R14" H 5368 5996 50  0000 L CNN
F 1 "10K" H 5368 5905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 5300 5950 50  0001 C CNN
F 3 "~" H 5300 5950 50  0001 C CNN
	1    5300 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R16
U 1 1 6081655D
P 5750 5950
F 0 "R16" H 5818 5996 50  0000 L CNN
F 1 "10K" H 5818 5905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 5750 5950 50  0001 C CNN
F 3 "~" H 5750 5950 50  0001 C CNN
	1    5750 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R6
U 1 1 60837A58
P 1300 5150
F 0 "R6" V 1095 5150 50  0000 C CNN
F 1 "60" V 1186 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 1300 5150 50  0001 C CNN
F 3 "~" H 1300 5150 50  0001 C CNN
	1    1300 5150
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R7
U 1 1 60838AAC
P 1300 5550
F 0 "R7" V 1095 5550 50  0000 C CNN
F 1 "5" V 1186 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 1300 5550 50  0001 C CNN
F 3 "~" H 1300 5550 50  0001 C CNN
	1    1300 5550
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R8
U 1 1 60839AEE
P 1300 5950
F 0 "R8" V 1095 5950 50  0000 C CNN
F 1 "5" V 1186 5950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 1300 5950 50  0001 C CNN
F 3 "~" H 1300 5950 50  0001 C CNN
	1    1300 5950
	0    1    1    0   
$EndComp
Wire Wire Line
	8950 6200 8950 6100
$Comp
L power:GND #PWR0144
U 1 1 608916CD
P 8950 6200
F 0 "#PWR0144" H 8950 5950 50  0001 C CNN
F 1 "GND" H 8955 6027 50  0000 C CNN
F 2 "" H 8950 6200 50  0001 C CNN
F 3 "" H 8950 6200 50  0001 C CNN
	1    8950 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 4900 6350 4550
Wire Wire Line
	6850 4900 6850 5050
Wire Wire Line
	6350 5050 6350 4900
Wire Wire Line
	6350 5400 6850 5400
Wire Wire Line
	6350 5250 6350 5400
Wire Wire Line
	6850 5400 6850 5250
$Comp
L Device:C_Small C?
U 1 1 6089E711
P 6850 5150
AR Path="/6093CD3D/6089E711" Ref="C?"  Part="1" 
AR Path="/6103AB31/6089E711" Ref="C33"  Part="1" 
F 0 "C33" H 6942 5196 50  0000 L CNN
F 1 "10uF" H 6942 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 6850 5150 50  0001 C CNN
F 3 "~" H 6850 5150 50  0001 C CNN
	1    6850 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 6089E717
P 6350 5150
AR Path="/6093CD3D/6089E717" Ref="C?"  Part="1" 
AR Path="/6103AB31/6089E717" Ref="C28"  Part="1" 
F 0 "C28" H 6442 5196 50  0000 L CNN
F 1 "100nF" H 6442 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 6350 5150 50  0001 C CNN
F 3 "~" H 6350 5150 50  0001 C CNN
	1    6350 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 5500 6350 5400
$Comp
L power:GND #PWR0147
U 1 1 6089E729
P 6350 5500
F 0 "#PWR0147" H 6350 5250 50  0001 C CNN
F 1 "GND" H 6355 5327 50  0000 C CNN
F 2 "" H 6350 5500 50  0001 C CNN
F 3 "" H 6350 5500 50  0001 C CNN
	1    6350 5500
	1    0    0    -1  
$EndComp
$Comp
L power:+2V5 #PWR0157
U 1 1 608C1A96
P 6350 4550
F 0 "#PWR0157" H 6350 4400 50  0001 C CNN
F 1 "+2V5" H 6365 4723 50  0000 C CNN
F 2 "" H 6350 4550 50  0001 C CNN
F 3 "" H 6350 4550 50  0001 C CNN
	1    6350 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 608F462A
P 10250 1600
AR Path="/6093CD3D/608F462A" Ref="C?"  Part="1" 
AR Path="/6103AB31/608F462A" Ref="C26"  Part="1" 
F 0 "C26" H 10342 1646 50  0000 L CNN
F 1 "100nF" H 10342 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 10250 1600 50  0001 C CNN
F 3 "~" H 10250 1600 50  0001 C CNN
	1    10250 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 1850 10050 1850
Wire Wire Line
	10250 1700 10250 1850
Wire Wire Line
	10250 1500 10250 1350
$Comp
L Device:C_Small C?
U 1 1 608FB0E7
P 10750 1600
AR Path="/6093CD3D/608FB0E7" Ref="C?"  Part="1" 
AR Path="/6103AB31/608FB0E7" Ref="C27"  Part="1" 
F 0 "C27" H 10842 1646 50  0000 L CNN
F 1 "100nF" H 10842 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 10750 1600 50  0001 C CNN
F 3 "~" H 10750 1600 50  0001 C CNN
	1    10750 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 1850 10750 1850
Wire Wire Line
	10750 1700 10750 1850
Wire Wire Line
	10750 1500 10750 1350
Wire Wire Line
	10050 1350 10250 1350
Connection ~ 10050 1350
Connection ~ 10250 1350
Wire Wire Line
	10250 1350 10750 1350
Connection ~ 9750 1850
Connection ~ 10250 1850
$Comp
L power:+3.3V #PWR0158
U 1 1 60936316
P 2300 2300
F 0 "#PWR0158" H 2300 2150 50  0001 C CNN
F 1 "+3.3V" H 2315 2473 50  0000 C CNN
F 2 "" H 2300 2300 50  0001 C CNN
F 3 "" H 2300 2300 50  0001 C CNN
	1    2300 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0170
U 1 1 60948DD7
P 5050 2500
F 0 "#PWR0170" H 5050 2350 50  0001 C CNN
F 1 "+3.3V" H 5065 2673 50  0000 C CNN
F 2 "" H 5050 2500 50  0001 C CNN
F 3 "" H 5050 2500 50  0001 C CNN
	1    5050 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 2250 8150 2600
$Comp
L power:+3.3V #PWR0171
U 1 1 6095FF62
P 8150 2250
F 0 "#PWR0171" H 8150 2100 50  0001 C CNN
F 1 "+3.3V" H 8165 2423 50  0000 C CNN
F 2 "" H 8150 2250 50  0001 C CNN
F 3 "" H 8150 2250 50  0001 C CNN
	1    8150 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0174
U 1 1 60965609
P 10050 2250
F 0 "#PWR0174" H 10050 2000 50  0001 C CNN
F 1 "GND" H 10055 2077 50  0000 C CNN
F 2 "" H 10050 2250 50  0001 C CNN
F 3 "" H 10050 2250 50  0001 C CNN
	1    10050 2250
	1    0    0    -1  
$EndComp
Connection ~ 10050 1850
Wire Wire Line
	10050 1850 10250 1850
Connection ~ 6850 4900
Connection ~ 6350 5400
Connection ~ 6350 4900
Wire Wire Line
	6350 4900 6850 4900
Connection ~ 8950 5600
Wire Wire Line
	8450 5600 8950 5600
$Comp
L Air_Pollution_Sensor-cache:LD1117S25TR_SOT223 U12
U 1 1 609A3081
P 1600 1100
F 0 "U12" H 1600 1342 50  0000 C CNN
F 1 "LD1117S25TR_SOT223" H 1600 1251 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 1600 1300 50  0001 C CNN
F 3 "" H 1700 850 50  0001 C CNN
	1    1600 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  1100 1300 1100
Wire Wire Line
	950  1150 950  1100
$Comp
L Device:C_Small C?
U 1 1 609B1410
P 2200 1250
AR Path="/6093CD3D/609B1410" Ref="C?"  Part="1" 
AR Path="/6103AB31/609B1410" Ref="C24"  Part="1" 
F 0 "C24" H 2292 1296 50  0000 L CNN
F 1 "10uF" H 2292 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2200 1250 50  0001 C CNN
F 3 "~" H 2200 1250 50  0001 C CNN
	1    2200 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 609B1416
P 950 1250
AR Path="/6093CD3D/609B1416" Ref="C?"  Part="1" 
AR Path="/6103AB31/609B1416" Ref="C23"  Part="1" 
F 0 "C23" H 1042 1296 50  0000 L CNN
F 1 "10uF" H 1042 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 950 1250 50  0001 C CNN
F 3 "~" H 950 1250 50  0001 C CNN
	1    950  1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0175
U 1 1 609B1428
P 950 1450
F 0 "#PWR0175" H 950 1200 50  0001 C CNN
F 1 "GND" H 955 1277 50  0000 C CNN
F 2 "" H 950 1450 50  0001 C CNN
F 3 "" H 950 1450 50  0001 C CNN
	1    950  1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0176
U 1 1 609B142E
P 1600 1450
F 0 "#PWR0176" H 1600 1200 50  0001 C CNN
F 1 "GND" H 1605 1277 50  0000 C CNN
F 2 "" H 1600 1450 50  0001 C CNN
F 3 "" H 1600 1450 50  0001 C CNN
	1    1600 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0177
U 1 1 609B1434
P 2200 1450
F 0 "#PWR0177" H 2200 1200 50  0001 C CNN
F 1 "GND" H 2205 1277 50  0000 C CNN
F 2 "" H 2200 1450 50  0001 C CNN
F 3 "" H 2200 1450 50  0001 C CNN
	1    2200 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+2V5 #PWR0178
U 1 1 609DEECB
P 2200 900
F 0 "#PWR0178" H 2200 750 50  0001 C CNN
F 1 "+2V5" H 2215 1073 50  0000 C CNN
F 2 "" H 2200 900 50  0001 C CNN
F 3 "" H 2200 900 50  0001 C CNN
	1    2200 900 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0179
U 1 1 609EBB0A
P 950 900
F 0 "#PWR0179" H 950 750 50  0001 C CNN
F 1 "+5V" H 965 1073 50  0000 C CNN
F 2 "" H 950 900 50  0001 C CNN
F 3 "" H 950 900 50  0001 C CNN
	1    950  900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 4600 4550 4600
Wire Wire Line
	3400 4700 4550 4700
$Comp
L Device:R_Small_US R12
U 1 1 60A3A5E4
P 4200 4400
F 0 "R12" V 3995 4400 50  0000 C CNN
F 1 "22" V 4086 4400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4200 4400 50  0001 C CNN
F 3 "~" H 4200 4400 50  0001 C CNN
	1    4200 4400
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R13
U 1 1 60A3BF30
P 4200 4500
F 0 "R13" V 3995 4500 50  0000 C CNN
F 1 "22" V 4086 4500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4200 4500 50  0001 C CNN
F 3 "~" H 4200 4500 50  0001 C CNN
	1    4200 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 4400 4550 4400
Wire Wire Line
	4300 4500 4550 4500
Wire Wire Line
	5850 3100 6300 3100
Text Label 6300 3100 2    50   ~ 0
CDONE
$Comp
L Device:R_Small_US R15
U 1 1 60AEB1AE
P 5750 3100
F 0 "R15" V 5545 3100 50  0000 C CNN
F 1 "510" V 5636 3100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 5750 3100 50  0001 C CNN
F 3 "~" H 5750 3100 50  0001 C CNN
	1    5750 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 3100 5550 3100
$Comp
L Device:LED D4
U 1 1 60AF91AE
P 6300 3400
F 0 "D4" V 6339 3282 50  0000 R CNN
F 1 "LED" V 6248 3282 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6300 3400 50  0001 C CNN
F 3 "~" H 6300 3400 50  0001 C CNN
	1    6300 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6300 3250 6300 3100
$Comp
L power:GND #PWR0185
U 1 1 60B058D2
P 6300 3800
F 0 "#PWR0185" H 6300 3550 50  0001 C CNN
F 1 "GND" H 6305 3627 50  0000 C CNN
F 2 "" H 6300 3800 50  0001 C CNN
F 3 "" H 6300 3800 50  0001 C CNN
	1    6300 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3550 6300 3800
Text Label 10700 5750 2    50   ~ 0
CRESET_B
Text HLabel 10000 5750 0    50   Input ~ 0
CRESET_B
$Comp
L Device:C_Small C?
U 1 1 60B3733E
P 3650 3250
AR Path="/6093CD3D/60B3733E" Ref="C?"  Part="1" 
AR Path="/6103AB31/60B3733E" Ref="C25"  Part="1" 
F 0 "C25" H 3742 3296 50  0000 L CNN
F 1 "1uF" H 3742 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3650 3250 50  0001 C CNN
F 3 "~" H 3650 3250 50  0001 C CNN
	1    3650 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0186
U 1 1 60B3C634
P 3650 3500
F 0 "#PWR0186" H 3650 3250 50  0001 C CNN
F 1 "GND" H 3655 3327 50  0000 C CNN
F 2 "" H 3650 3500 50  0001 C CNN
F 3 "" H 3650 3500 50  0001 C CNN
	1    3650 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R9
U 1 1 60B4A51D
P 3500 3100
F 0 "R9" V 3295 3100 50  0000 C CNN
F 1 "4.7K" V 3386 3100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 3500 3100 50  0001 C CNN
F 3 "~" H 3500 3100 50  0001 C CNN
	1    3500 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 3100 3350 2650
$Comp
L power:+2V5 #PWR0187
U 1 1 60B5A964
P 3350 2650
F 0 "#PWR0187" H 3350 2500 50  0001 C CNN
F 1 "+2V5" H 3365 2823 50  0000 C CNN
F 2 "" H 3350 2650 50  0001 C CNN
F 3 "" H 3350 2650 50  0001 C CNN
	1    3350 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 3150 3650 3100
Wire Wire Line
	3650 3500 3650 3350
Wire Wire Line
	3350 3100 3400 3100
Wire Wire Line
	3650 3100 3600 3100
Wire Wire Line
	3650 3100 4550 3100
Connection ~ 3650 3100
Text Label 6950 1200 2    50   ~ 0
SF_SPI_SCK
Text Label 6950 1100 2    50   ~ 0
SF_SPI_SS
Text Label 6950 1300 2    50   ~ 0
SF_SPI_DI
Text Label 6950 1400 2    50   ~ 0
SF_SPI_DO
Wire Wire Line
	6950 1200 5800 1200
Wire Wire Line
	6950 1100 5800 1100
Wire Wire Line
	5800 1300 6950 1300
Wire Wire Line
	5800 1400 6950 1400
$Comp
L power:GND #PWR0194
U 1 1 609941D7
P 5950 1550
F 0 "#PWR0194" H 5950 1300 50  0001 C CNN
F 1 "GND" H 5955 1377 50  0000 C CNN
F 2 "" H 5950 1550 50  0001 C CNN
F 3 "" H 5950 1550 50  0001 C CNN
	1    5950 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 1550 5950 1500
Wire Wire Line
	5950 1500 5800 1500
$Comp
L power:+3.3V #PWR0195
U 1 1 609A71AF
P 4950 1000
F 0 "#PWR0195" H 4950 850 50  0001 C CNN
F 1 "+3.3V" H 4965 1173 50  0000 C CNN
F 2 "" H 4950 1000 50  0001 C CNN
F 3 "" H 4950 1000 50  0001 C CNN
	1    4950 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 1000 4950 1100
$Comp
L Air_Pollution_Sensor-rescue:61301021821-Air_Pollution_Sensor-cache J?
U 1 1 60B19032
P 5500 1300
AR Path="/60B19032" Ref="J?"  Part="1" 
AR Path="/6103AB31/60B19032" Ref="J4"  Part="1" 
F 0 "J4" H 5500 1767 50  0000 C CNN
F 1 "61301021821" H 5500 1676 50  0000 C CNN
F 2 "Air_Pollution_Sensor:61301021821" H 5500 1300 50  0001 L BNN
F 3 "" H 5500 1300 50  0001 L BNN
	1    5500 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1100 4950 1100
$Comp
L power:GND #PWR0197
U 1 1 60B505E9
P 5100 1650
F 0 "#PWR0197" H 5100 1400 50  0001 C CNN
F 1 "GND" H 5105 1477 50  0000 C CNN
F 2 "" H 5100 1650 50  0001 C CNN
F 3 "" H 5100 1650 50  0001 C CNN
	1    5100 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1200 5100 1200
Wire Wire Line
	5100 1200 5100 1300
Wire Wire Line
	5200 1500 5100 1500
Connection ~ 5100 1500
Wire Wire Line
	5100 1500 5100 1650
Wire Wire Line
	5200 1400 5100 1400
Connection ~ 5100 1400
Wire Wire Line
	5100 1400 5100 1500
Wire Wire Line
	5200 1300 5100 1300
Connection ~ 5100 1300
Wire Wire Line
	5100 1300 5100 1400
$Comp
L power:+1V2 #PWR0129
U 1 1 610ABFDC
P 7550 4400
F 0 "#PWR0129" H 7550 4250 50  0001 C CNN
F 1 "+1V2" H 7565 4573 50  0000 C CNN
F 2 "" H 7550 4400 50  0001 C CNN
F 3 "" H 7550 4400 50  0001 C CNN
	1    7550 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 5350 10700 5350
$Comp
L Connector:TestPoint TP2
U 1 1 608422A7
P 10700 5200
F 0 "TP2" H 10758 5318 50  0000 L CNN
F 1 "TestPoint" H 10758 5227 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm" H 10900 5200 50  0001 C CNN
F 3 "~" H 10900 5200 50  0001 C CNN
	1    10700 5200
	1    0    0    -1  
$EndComp
Text Label 10000 5350 0    50   ~ 0
ICE_CLK
Connection ~ 8950 6100
Wire Wire Line
	8650 4500 8650 4650
Wire Wire Line
	8150 4650 8150 4500
Wire Wire Line
	8150 4850 8150 5000
$Comp
L Device:C_Small C?
U 1 1 6081A364
P 8650 4750
AR Path="/6093CD3D/6081A364" Ref="C?"  Part="1" 
AR Path="/6103AB31/6081A364" Ref="C32"  Part="1" 
F 0 "C32" H 8742 4796 50  0000 L CNN
F 1 "1uF" H 8742 4705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8650 4750 50  0001 C CNN
F 3 "~" H 8650 4750 50  0001 C CNN
	1    8650 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 6081A36A
P 8150 4750
AR Path="/6093CD3D/6081A36A" Ref="C?"  Part="1" 
AR Path="/6103AB31/6081A36A" Ref="C31"  Part="1" 
F 0 "C31" H 8242 4796 50  0000 L CNN
F 1 "10nF" H 8242 4705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8150 4750 50  0001 C CNN
F 3 "~" H 8150 4750 50  0001 C CNN
	1    8150 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 4500 8150 4500
Wire Wire Line
	8650 5000 8150 5000
$Comp
L power:GND #PWR0198
U 1 1 60869ACE
P 8650 5050
F 0 "#PWR0198" H 8650 4800 50  0001 C CNN
F 1 "GND" H 8655 4877 50  0000 C CNN
F 2 "" H 8650 5050 50  0001 C CNN
F 3 "" H 8650 5050 50  0001 C CNN
	1    8650 5050
	1    0    0    -1  
$EndComp
Text Notes 7350 7500 0    50   ~ 10
FPGA_ICE40UP5K_module
Wire Wire Line
	7650 4900 7650 5000
Connection ~ 8150 4500
Wire Wire Line
	6850 4900 7450 4900
Wire Wire Line
	7450 4900 7450 5000
Wire Wire Line
	8650 4850 8650 5000
Connection ~ 8650 5000
Wire Wire Line
	8650 5000 8650 5050
Wire Wire Line
	7650 4900 8000 4900
Wire Wire Line
	8000 4900 8000 5600
Wire Wire Line
	8000 5600 8450 5600
Connection ~ 8450 5600
Wire Wire Line
	7550 4500 8150 4500
Wire Wire Line
	7550 4500 7550 5000
Wire Wire Line
	7550 4500 7550 4400
Connection ~ 7550 4500
Wire Wire Line
	5050 2500 5050 2700
Wire Wire Line
	10000 5750 10700 5750
Wire Wire Line
	10000 5650 10700 5650
Wire Wire Line
	10000 5450 10700 5450
Wire Wire Line
	10000 5550 10700 5550
Wire Wire Line
	10700 5200 10700 5350
Wire Wire Line
	10050 1850 10050 2250
Wire Wire Line
	2200 1100 1900 1100
Wire Wire Line
	2200 900  2200 1100
Wire Wire Line
	950  900  950  1100
Connection ~ 950  1100
Wire Wire Line
	950  1450 950  1350
Wire Wire Line
	1600 1450 1600 1400
Wire Wire Line
	2200 1450 2200 1350
Wire Wire Line
	2200 1150 2200 1100
Connection ~ 2200 1100
Wire Wire Line
	2300 2300 2300 2450
Text Notes 8200 7650 0    50   ~ 10
2021. 04. 13
Text Notes 10600 7650 0    50   ~ 10
1.0
$EndSCHEMATC
