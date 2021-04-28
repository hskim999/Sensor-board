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
L FPGA_Lattice:ICE40UP5K-SG48ITR U?
U 1 1 61070DEB
P 2200 3200
AR Path="/61070DEB" Ref="U?"  Part="1" 
AR Path="/6093CD3D/61070DEB" Ref="U?"  Part="1" 
AR Path="/6103AB31/61070DEB" Ref="U2"  Part="1" 
F 0 "U2" H 2530 3253 50  0000 L CNN
F 1 "ICE40UP5K-SG48ITR" H 2530 3162 50  0000 L CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm" H 2200 1850 50  0001 C CNN
F 3 "http://www.latticesemi.com/Products/FPGAandCPLD/iCE40Ultra" H 1800 4200 50  0001 C CNN
	1    2200 3200
	1    0    0    -1  
$EndComp
Text HLabel 10550 5400 2    50   Input ~ 0
SCL
Text HLabel 10550 5300 2    50   3State ~ 0
SDA
Connection ~ 2600 5600
Wire Wire Line
	2600 5600 2600 5800
$Comp
L FPGA_Lattice:ICE40UP5K-SG48ITR U?
U 3 1 61070E3F
P 8950 3100
AR Path="/61070E3F" Ref="U?"  Part="3" 
AR Path="/6093CD3D/61070E3F" Ref="U?"  Part="3" 
AR Path="/6103AB31/61070E3F" Ref="U2"  Part="3" 
F 0 "U2" H 9280 3203 50  0000 L CNN
F 1 "ICE40UP5K-SG48ITR" H 9280 3112 50  0000 L CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm" H 8950 1750 50  0001 C CNN
F 3 "http://www.latticesemi.com/Products/FPGAandCPLD/iCE40Ultra" H 8550 4100 50  0001 C CNN
	3    8950 3100
	1    0    0    -1  
$EndComp
$Comp
L Memory_Flash:W25Q32JVSS U?
U 1 1 61070E4B
P 4850 6800
AR Path="/6093CD3D/61070E4B" Ref="U?"  Part="1" 
AR Path="/6103AB31/61070E4B" Ref="U7"  Part="1" 
F 0 "U7" H 4850 7381 50  0000 C CNN
F 1 "W25Q32JVSS" H 4850 7290 50  0000 C CNN
F 2 "Package_SO:SOIC-8_5.23x5.23mm_P1.27mm" H 4850 6800 50  0001 C CNN
F 3 "http://www.winbond.com/resource-files/w25q32jv%20revg%2003272018%20plus.pdf" H 4850 6800 50  0001 C CNN
	1    4850 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 7200 4850 7450
Wire Wire Line
	4350 6900 3500 6900
$Comp
L Device:R_Small R?
U 1 1 61070E5A
P 4150 6250
AR Path="/6093CD3D/61070E5A" Ref="R?"  Part="1" 
AR Path="/6103AB31/61070E5A" Ref="R7"  Part="1" 
F 0 "R7" H 4209 6296 50  0000 L CNN
F 1 "10K" H 4209 6205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4150 6250 50  0001 C CNN
F 3 "~" H 4150 6250 50  0001 C CNN
	1    4150 6250
	1    0    0    -1  
$EndComp
Connection ~ 4850 5900
Wire Wire Line
	4150 6350 4150 6700
Text Label 3500 6700 0    50   ~ 0
SF_SPI_SS
Text Label 3500 6900 0    50   ~ 0
SF_SPI_SCK
$Comp
L Device:R_Small R?
U 1 1 61070E66
P 4150 7150
AR Path="/6093CD3D/61070E66" Ref="R?"  Part="1" 
AR Path="/6103AB31/61070E66" Ref="R8"  Part="1" 
F 0 "R8" H 4209 7196 50  0000 L CNN
F 1 "1K" H 4209 7105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 4150 7150 50  0001 C CNN
F 3 "~" H 4150 7150 50  0001 C CNN
	1    4150 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 7450 4150 7250
Wire Wire Line
	4150 6700 4150 7050
$Comp
L Device:C_Small C?
U 1 1 61070E74
P 5500 7150
AR Path="/6093CD3D/61070E74" Ref="C?"  Part="1" 
AR Path="/6103AB31/61070E74" Ref="C22"  Part="1" 
F 0 "C22" H 5592 7196 50  0000 L CNN
F 1 "100nF" H 5592 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5500 7150 50  0001 C CNN
F 3 "~" H 5500 7150 50  0001 C CNN
	1    5500 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 7450 5500 7250
Wire Wire Line
	4150 6150 4150 5900
Wire Wire Line
	4850 5900 4850 6100
Connection ~ 4150 6700
Wire Wire Line
	4150 6700 4350 6700
Wire Wire Line
	4850 5900 4150 5900
Wire Wire Line
	3500 6700 4150 6700
Wire Wire Line
	6200 5900 6200 6150
Wire Wire Line
	5750 5900 6200 5900
Connection ~ 5750 5900
Wire Wire Line
	5750 6150 5750 5900
Text Label 6700 6700 2    50   ~ 0
SF_SPI_DO
Text Label 6700 6600 2    50   ~ 0
SF_SPI_DI
Text Label 6700 6900 2    50   ~ 0
SST_WP
Wire Wire Line
	5750 6900 6700 6900
Connection ~ 6200 7000
Wire Wire Line
	6200 7000 6700 7000
Text Label 6700 7000 2    50   ~ 0
SST_HOLD
Wire Wire Line
	6200 6350 6200 7000
Connection ~ 5750 6900
Wire Wire Line
	5750 6350 5750 6900
$Comp
L Device:R_Small R?
U 1 1 61070E95
P 6200 6250
AR Path="/6093CD3D/61070E95" Ref="R?"  Part="1" 
AR Path="/6103AB31/61070E95" Ref="R11"  Part="1" 
F 0 "R11" H 6259 6296 50  0000 L CNN
F 1 "10K" H 6259 6205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 6200 6250 50  0001 C CNN
F 3 "~" H 6200 6250 50  0001 C CNN
	1    6200 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 61070E9B
P 5750 6250
AR Path="/6093CD3D/61070E9B" Ref="R?"  Part="1" 
AR Path="/6103AB31/61070E9B" Ref="R10"  Part="1" 
F 0 "R10" H 5809 6296 50  0000 L CNN
F 1 "10K" H 5809 6205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 5750 6250 50  0001 C CNN
F 3 "~" H 5750 6250 50  0001 C CNN
	1    5750 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 6600 6700 6600
Wire Wire Line
	5350 6700 6700 6700
Wire Wire Line
	5350 7000 6200 7000
Wire Wire Line
	5350 6900 5750 6900
Wire Wire Line
	4850 5900 5750 5900
Wire Wire Line
	5500 6100 4850 6100
Wire Wire Line
	5500 6100 5500 7050
Connection ~ 4850 6100
Wire Wire Line
	4850 6100 4850 6400
Wire Wire Line
	4300 5200 5000 5200
Wire Wire Line
	4300 5100 5000 5100
Wire Wire Line
	4300 3600 5000 3600
$Comp
L FPGA_Lattice:ICE40UP5K-SG48ITR U?
U 2 1 61070F1E
P 5500 4300
AR Path="/61070F1E" Ref="U?"  Part="2" 
AR Path="/6093CD3D/61070F1E" Ref="U?"  Part="2" 
AR Path="/6103AB31/61070F1E" Ref="U2"  Part="2" 
F 0 "U2" H 5500 3225 50  0000 C CNN
F 1 "ICE40UP5K-SG48ITR" H 5500 3134 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm" H 5500 2950 50  0001 C CNN
F 3 "http://www.latticesemi.com/Products/FPGAandCPLD/iCE40Ultra" H 5100 5300 50  0001 C CNN
	2    5500 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3600 6450 3600
Text Label 4300 5100 0    50   ~ 0
SF_SPI_SCK
Text Label 4300 5200 0    50   ~ 0
SF_SPI_SS
Text Label 6450 3600 2    50   ~ 0
CDONE
Text Label 4300 3600 0    50   ~ 0
CRESET_B
Text HLabel 4300 3600 0    50   Input ~ 0
CRESET_B
Text HLabel 6450 3600 2    50   Input ~ 0
CDONE
$Comp
L Device:C_Small C?
U 1 1 61070F70
P 9150 2100
AR Path="/6093CD3D/61070F70" Ref="C?"  Part="1" 
AR Path="/6103AB31/61070F70" Ref="C36"  Part="1" 
F 0 "C36" H 9242 2146 50  0000 L CNN
F 1 "1uF" H 9242 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9150 2100 50  0001 C CNN
F 3 "~" H 9150 2100 50  0001 C CNN
	1    9150 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 61070F76
P 9650 2100
AR Path="/6093CD3D/61070F76" Ref="C?"  Part="1" 
AR Path="/6103AB31/61070F76" Ref="C38"  Part="1" 
F 0 "C38" H 9742 2146 50  0000 L CNN
F 1 "100nF" H 9742 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9650 2100 50  0001 C CNN
F 3 "~" H 9650 2100 50  0001 C CNN
	1    9650 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 61070F7C
P 10150 2100
AR Path="/6093CD3D/61070F7C" Ref="C?"  Part="1" 
AR Path="/6103AB31/61070F7C" Ref="C40"  Part="1" 
F 0 "C40" H 10242 2146 50  0000 L CNN
F 1 "10nF" H 10242 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 10150 2100 50  0001 C CNN
F 3 "~" H 10150 2100 50  0001 C CNN
	1    10150 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 2200 9150 2350
Wire Wire Line
	9150 2350 9650 2350
Wire Wire Line
	10150 2350 10150 2200
Wire Wire Line
	9650 2200 9650 2350
Connection ~ 9650 2350
Wire Wire Line
	9650 2350 10150 2350
Wire Wire Line
	9150 2000 9150 1850
Wire Wire Line
	9150 1850 9500 1850
Wire Wire Line
	9650 2000 9650 1850
Connection ~ 9650 1850
Wire Wire Line
	10150 1850 10150 2000
$Comp
L Device:R_Small R?
U 1 1 61070F8D
P 10600 1850
AR Path="/61070F8D" Ref="R?"  Part="1" 
AR Path="/6093CD3D/61070F8D" Ref="R?"  Part="1" 
AR Path="/6103AB31/61070F8D" Ref="R15"  Part="1" 
F 0 "R15" V 10404 1850 50  0000 C CNN
F 1 "1" V 10495 1850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 10600 1850 50  0001 C CNN
F 3 "~" H 10600 1850 50  0001 C CNN
	1    10600 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	10150 1850 10250 1850
Connection ~ 10150 1850
Wire Wire Line
	10700 1850 10800 1850
Wire Wire Line
	11000 1500 11000 1850
$Comp
L Device:C_Small C?
U 1 1 61070F9C
P 1450 1550
AR Path="/6093CD3D/61070F9C" Ref="C?"  Part="1" 
AR Path="/6103AB31/61070F9C" Ref="C17"  Part="1" 
F 0 "C17" H 1542 1596 50  0000 L CNN
F 1 "1uF" H 1542 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1450 1550 50  0001 C CNN
F 3 "~" H 1450 1550 50  0001 C CNN
	1    1450 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 61070FA2
P 1950 1550
AR Path="/6093CD3D/61070FA2" Ref="C?"  Part="1" 
AR Path="/6103AB31/61070FA2" Ref="C19"  Part="1" 
F 0 "C19" H 2042 1596 50  0000 L CNN
F 1 "100nF" H 2042 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1950 1550 50  0001 C CNN
F 3 "~" H 1950 1550 50  0001 C CNN
	1    1950 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 1650 1450 1800
Wire Wire Line
	1450 1800 1950 1800
Wire Wire Line
	1950 1650 1950 1800
Connection ~ 1950 1800
Wire Wire Line
	1950 1800 2200 1800
Wire Wire Line
	1450 1450 1450 1300
Wire Wire Line
	1450 1300 1950 1300
Wire Wire Line
	1950 1450 1950 1300
Connection ~ 1950 1300
Wire Wire Line
	1800 3500 1100 3500
Wire Wire Line
	1800 2900 1100 2900
Wire Wire Line
	1800 2800 1100 2800
Wire Wire Line
	1800 2700 1100 2700
Wire Wire Line
	1800 2600 1100 2600
Wire Wire Line
	8950 2400 8950 1850
Wire Wire Line
	8950 1850 9150 1850
Connection ~ 9150 1850
Text Label 1100 3500 0    50   ~ 0
IOT_46B_G0
Text HLabel 1100 2700 0    50   Output ~ 0
spi_mosi
Text HLabel 1100 2800 0    50   Input ~ 0
spi_miso
Text HLabel 1100 2900 0    50   Output ~ 0
spi_sck
Text HLabel 1100 2600 0    50   Output ~ 0
spi_cs
Text HLabel 1100 3500 0    50   Input ~ 0
ICE_CLK
Wire Wire Line
	9650 1850 10150 1850
$Comp
L Connector:TestPoint TP?
U 1 1 6107100E
P 10800 1850
AR Path="/6093CD3D/6107100E" Ref="TP?"  Part="1" 
AR Path="/6103AB31/6107100E" Ref="TP9"  Part="1" 
F 0 "TP9" H 10858 1968 50  0000 L CNN
F 1 "DNI" H 10858 1877 50  0000 L CNN
F 2 "" H 11000 1850 50  0001 C CNN
F 3 "~" H 11000 1850 50  0001 C CNN
	1    10800 1850
	1    0    0    -1  
$EndComp
Connection ~ 10800 1850
Wire Wire Line
	10800 1850 11000 1850
$Comp
L Connector:TestPoint TP?
U 1 1 61071016
P 10250 1850
AR Path="/6093CD3D/61071016" Ref="TP?"  Part="1" 
AR Path="/6103AB31/61071016" Ref="TP7"  Part="1" 
F 0 "TP7" H 10308 1968 50  0000 L CNN
F 1 "DNI" H 10308 1877 50  0000 L CNN
F 2 "" H 10450 1850 50  0001 C CNN
F 3 "~" H 10450 1850 50  0001 C CNN
	1    10250 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 1300 2250 1300
Wire Wire Line
	9500 1850 9500 1500
Connection ~ 9500 1850
Wire Wire Line
	9500 1850 9650 1850
$Comp
L Device:C_Small C?
U 1 1 61071049
P 7550 5450
AR Path="/6093CD3D/61071049" Ref="C?"  Part="1" 
AR Path="/6103AB31/61071049" Ref="C33"  Part="1" 
F 0 "C33" H 7642 5496 50  0000 L CNN
F 1 "100nF" H 7642 5405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 7550 5450 50  0001 C CNN
F 3 "~" H 7550 5450 50  0001 C CNN
	1    7550 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 5250 7850 5250
$Comp
L Device:C_Small C?
U 1 1 61071095
P 7350 4550
AR Path="/6093CD3D/61071095" Ref="C?"  Part="1" 
AR Path="/6103AB31/61071095" Ref="C32"  Part="1" 
F 0 "C32" H 7442 4596 50  0000 L CNN
F 1 "1uF" H 7442 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 7350 4550 50  0001 C CNN
F 3 "~" H 7350 4550 50  0001 C CNN
	1    7350 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 6107109B
P 7850 4550
AR Path="/6093CD3D/6107109B" Ref="C?"  Part="1" 
AR Path="/6103AB31/6107109B" Ref="C34"  Part="1" 
F 0 "C34" H 7942 4596 50  0000 L CNN
F 1 "100nF" H 7942 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 7850 4550 50  0001 C CNN
F 3 "~" H 7850 4550 50  0001 C CNN
	1    7850 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 610710A1
P 8350 4550
AR Path="/6093CD3D/610710A1" Ref="C?"  Part="1" 
AR Path="/6103AB31/610710A1" Ref="C35"  Part="1" 
F 0 "C35" H 8442 4596 50  0000 L CNN
F 1 "10nF" H 8442 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8350 4550 50  0001 C CNN
F 3 "~" H 8350 4550 50  0001 C CNN
	1    8350 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 610710B1
P 6900 4400
AR Path="/610710B1" Ref="R?"  Part="1" 
AR Path="/6093CD3D/610710B1" Ref="R?"  Part="1" 
AR Path="/6103AB31/610710B1" Ref="R12"  Part="1" 
F 0 "R12" V 6704 4400 50  0000 C CNN
F 1 "1" V 6795 4400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 6900 4400 50  0001 C CNN
F 3 "~" H 6900 4400 50  0001 C CNN
	1    6900 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	7000 4400 7100 4400
$Comp
L Connector:TestPoint TP?
U 1 1 610710BB
P 7100 4400
AR Path="/6093CD3D/610710BB" Ref="TP?"  Part="1" 
AR Path="/6103AB31/610710BB" Ref="TP4"  Part="1" 
F 0 "TP4" H 7158 4518 50  0000 L CNN
F 1 "DNI" H 7158 4427 50  0000 L CNN
F 2 "" H 7300 4400 50  0001 C CNN
F 3 "~" H 7300 4400 50  0001 C CNN
	1    7100 4400
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP?
U 1 1 610710C1
P 6600 4400
AR Path="/6093CD3D/610710C1" Ref="TP?"  Part="1" 
AR Path="/6103AB31/610710C1" Ref="TP3"  Part="1" 
F 0 "TP3" H 6658 4518 50  0000 L CNN
F 1 "DNI" H 6658 4427 50  0000 L CNN
F 2 "" H 6800 4400 50  0001 C CNN
F 3 "~" H 6800 4400 50  0001 C CNN
	1    6600 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 4400 8100 4050
Connection ~ 7100 4400
Wire Wire Line
	6450 4400 6450 4050
Connection ~ 10250 1850
Connection ~ 8100 4400
Wire Wire Line
	6450 4400 6600 4400
Connection ~ 6600 4400
Wire Wire Line
	10550 5400 10200 5400
Wire Wire Line
	10550 5300 10200 5300
Text Label 10350 5400 2    50   ~ 0
SCL
Text Label 10350 5300 2    50   ~ 0
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
	2250 950  2250 1300
$Comp
L power:+3.3V #PWR0131
U 1 1 6119FEE0
P 11000 1500
F 0 "#PWR0131" H 11000 1350 50  0001 C CNN
F 1 "+3.3V" H 11015 1673 50  0000 C CNN
F 2 "" H 11000 1500 50  0001 C CNN
F 3 "" H 11000 1500 50  0001 C CNN
	1    11000 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0132
U 1 1 611F1D2C
P 4850 5650
F 0 "#PWR0132" H 4850 5500 50  0001 C CNN
F 1 "+3.3V" H 4865 5823 50  0000 C CNN
F 2 "" H 4850 5650 50  0001 C CNN
F 3 "" H 4850 5650 50  0001 C CNN
	1    4850 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 5650 4850 5900
Wire Wire Line
	9450 4400 9450 4050
Wire Wire Line
	8750 4400 8900 4400
$Comp
L Device:R_Small R?
U 1 1 610710D9
P 9000 4400
AR Path="/610710D9" Ref="R?"  Part="1" 
AR Path="/6093CD3D/610710D9" Ref="R?"  Part="1" 
AR Path="/6103AB31/610710D9" Ref="R14"  Part="1" 
F 0 "R14" V 8804 4400 50  0000 C CNN
F 1 "100" V 8895 4400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 9000 4400 50  0001 C CNN
F 3 "~" H 9000 4400 50  0001 C CNN
	1    9000 4400
	0    1    1    0   
$EndComp
Connection ~ 10300 4400
$Comp
L Connector:TestPoint TP?
U 1 1 6107108B
P 10300 4400
AR Path="/6093CD3D/6107108B" Ref="TP?"  Part="1" 
AR Path="/6103AB31/6107108B" Ref="TP8"  Part="1" 
F 0 "TP8" H 10358 4518 50  0000 L CNN
F 1 "DNI" H 10358 4427 50  0000 L CNN
F 2 "" H 10500 4400 50  0001 C CNN
F 3 "~" H 10500 4400 50  0001 C CNN
	1    10300 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10850 4400 11050 4400
Connection ~ 10850 4400
$Comp
L Connector:TestPoint TP?
U 1 1 61071083
P 10850 4400
AR Path="/6093CD3D/61071083" Ref="TP?"  Part="1" 
AR Path="/6103AB31/61071083" Ref="TP10"  Part="1" 
F 0 "TP10" H 10908 4518 50  0000 L CNN
F 1 "DNI" H 10908 4427 50  0000 L CNN
F 2 "" H 11050 4400 50  0001 C CNN
F 3 "~" H 11050 4400 50  0001 C CNN
	1    10850 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	11050 4050 11050 4400
Wire Wire Line
	10750 4400 10850 4400
$Comp
L Device:R_Small R?
U 1 1 61071077
P 10650 4400
AR Path="/61071077" Ref="R?"  Part="1" 
AR Path="/6093CD3D/61071077" Ref="R?"  Part="1" 
AR Path="/6103AB31/61071077" Ref="R16"  Part="1" 
F 0 "R16" V 10454 4400 50  0000 C CNN
F 1 "1" V 10545 4400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 10650 4400 50  0001 C CNN
F 3 "~" H 10650 4400 50  0001 C CNN
	1    10650 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	9950 4400 9950 4550
Wire Wire Line
	9450 4550 9450 4400
Wire Wire Line
	9450 4900 9950 4900
Wire Wire Line
	9450 4750 9450 4900
Wire Wire Line
	9950 4900 9950 4750
$Comp
L Device:C_Small C?
U 1 1 61071067
P 9950 4650
AR Path="/6093CD3D/61071067" Ref="C?"  Part="1" 
AR Path="/6103AB31/61071067" Ref="C41"  Part="1" 
F 0 "C41" H 10042 4696 50  0000 L CNN
F 1 "10uF" H 10042 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9950 4650 50  0001 C CNN
F 3 "~" H 9950 4650 50  0001 C CNN
	1    9950 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 61071061
P 9450 4650
AR Path="/6093CD3D/61071061" Ref="C?"  Part="1" 
AR Path="/6103AB31/61071061" Ref="C39"  Part="1" 
F 0 "C39" H 9542 4696 50  0000 L CNN
F 1 "100nF" H 9542 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9450 4650 50  0001 C CNN
F 3 "~" H 9450 4650 50  0001 C CNN
	1    9450 4650
	1    0    0    -1  
$EndComp
$Comp
L FPGA_Lattice:ICE40UP5K-SG48ITR U?
U 4 1 61070E39
P 8650 5750
AR Path="/61070E39" Ref="U?"  Part="4" 
AR Path="/6093CD3D/61070E39" Ref="U?"  Part="4" 
AR Path="/6103AB31/61070E39" Ref="U2"  Part="4" 
F 0 "U2" H 8880 5796 50  0000 L CNN
F 1 "ICE40UP5K-SG48ITR" H 8880 5705 50  0000 L CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm" H 8650 4400 50  0001 C CNN
F 3 "http://www.latticesemi.com/Products/FPGAandCPLD/iCE40Ultra" H 8250 6750 50  0001 C CNN
	4    8650 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 4400 7350 4400
Wire Wire Line
	8100 4400 8350 4400
Wire Wire Line
	7350 4450 7350 4400
Connection ~ 7350 4400
Wire Wire Line
	7350 4400 7850 4400
Wire Wire Line
	7850 4450 7850 4400
Connection ~ 7850 4400
Wire Wire Line
	7850 4400 8100 4400
Wire Wire Line
	8350 4400 8350 4450
Connection ~ 8350 4400
Wire Wire Line
	8350 4400 8650 4400
Wire Wire Line
	7350 4650 7350 4700
Wire Wire Line
	7350 4700 7850 4700
Wire Wire Line
	8350 4700 8350 4650
Wire Wire Line
	7850 4650 7850 4700
Connection ~ 7850 4700
Wire Wire Line
	7850 4700 8350 4700
$Comp
L power:+3.3V #PWR0133
U 1 1 612D578E
P 8100 4050
F 0 "#PWR0133" H 8100 3900 50  0001 C CNN
F 1 "+3.3V" H 8115 4223 50  0000 C CNN
F 2 "" H 8100 4050 50  0001 C CNN
F 3 "" H 8100 4050 50  0001 C CNN
	1    8100 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0134
U 1 1 612DF004
P 7550 5050
F 0 "#PWR0134" H 7550 4900 50  0001 C CNN
F 1 "+3.3V" H 7565 5223 50  0000 C CNN
F 2 "" H 7550 5050 50  0001 C CNN
F 3 "" H 7550 5050 50  0001 C CNN
	1    7550 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:D D?
U 1 1 61071041
P 8000 5250
AR Path="/6093CD3D/61071041" Ref="D?"  Part="1" 
AR Path="/6103AB31/61071041" Ref="D4"  Part="1" 
F 0 "D4" H 8000 5033 50  0000 C CNN
F 1 "D" H 8000 5124 50  0000 C CNN
F 2 "Diode_SMD:D_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8000 5250 50  0001 C CNN
F 3 "~" H 8000 5250 50  0001 C CNN
	1    8000 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	7550 5050 7550 5250
Wire Wire Line
	7550 5650 7550 5550
Wire Wire Line
	7550 5350 7550 5250
Connection ~ 7550 5250
Wire Wire Line
	8650 6250 8650 6150
Wire Wire Line
	8550 5350 8550 5250
$Comp
L power:+1V2 #PWR0137
U 1 1 613D0DC1
P 6450 4050
F 0 "#PWR0137" H 6450 3900 50  0001 C CNN
F 1 "+1V2" H 6465 4223 50  0000 C CNN
F 2 "" H 6450 4050 50  0001 C CNN
F 3 "" H 6450 4050 50  0001 C CNN
	1    6450 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+1V2 #PWR0138
U 1 1 613D198F
P 9450 4050
F 0 "#PWR0138" H 9450 3900 50  0001 C CNN
F 1 "+1V2" H 9465 4223 50  0000 C CNN
F 2 "" H 9450 4050 50  0001 C CNN
F 3 "" H 9450 4050 50  0001 C CNN
	1    9450 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+1V2 #PWR0139
U 1 1 613DB84D
P 11050 4050
F 0 "#PWR0139" H 11050 3900 50  0001 C CNN
F 1 "+1V2" H 11065 4223 50  0000 C CNN
F 2 "" H 11050 4050 50  0001 C CNN
F 3 "" H 11050 4050 50  0001 C CNN
	1    11050 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0140
U 1 1 613F08F8
P 9500 1500
F 0 "#PWR0140" H 9500 1350 50  0001 C CNN
F 1 "+3.3V" H 9515 1673 50  0000 C CNN
F 2 "" H 9500 1500 50  0001 C CNN
F 3 "" H 9500 1500 50  0001 C CNN
	1    9500 1500
	1    0    0    -1  
$EndComp
Connection ~ 9450 4400
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
Wire Wire Line
	9100 4400 9450 4400
Connection ~ 9950 4400
Wire Wire Line
	9450 4400 9950 4400
Wire Wire Line
	9950 4400 10300 4400
Wire Wire Line
	8650 4400 8650 5350
Wire Wire Line
	8750 4400 8750 5350
Wire Wire Line
	8150 5250 8550 5250
Wire Wire Line
	10300 4400 10550 4400
Wire Wire Line
	10250 1850 10500 1850
Wire Wire Line
	6600 4400 6800 4400
Text HLabel 10550 5500 2    50   Input ~ 0
temp_alert
Text Label 10100 5500 0    50   ~ 0
temp_alert
Wire Wire Line
	10050 5500 10550 5500
Text Label 4350 3900 0    50   ~ 0
temp_alert
Wire Wire Line
	4300 3900 5000 3900
Wire Wire Line
	8550 2900 7850 2900
Wire Wire Line
	8550 3000 7850 3000
Text Label 7850 2900 0    50   ~ 0
SDA
Text Label 7850 3000 0    50   ~ 0
SCL
Text HLabel 10250 6100 0    50   Input ~ 0
jtag_tck
Text HLabel 10250 6200 0    50   Input ~ 0
jtag_tdo
Text HLabel 10250 6300 0    50   Input ~ 0
jtag_tdi
Text HLabel 10250 6400 0    50   Input ~ 0
jtag_tms
Wire Wire Line
	1800 3400 1100 3400
Wire Wire Line
	1800 3600 1100 3600
Wire Wire Line
	1800 3800 1100 3800
Wire Wire Line
	1800 3900 1100 3900
Text Label 1100 3400 0    50   ~ 0
jtag_tck
Text Label 1100 3600 0    50   ~ 0
jtag_tdo
Text Label 1100 3800 0    50   ~ 0
jtag_tdi
Text Label 1100 3900 0    50   ~ 0
jtag_tms
Wire Wire Line
	10250 6400 10950 6400
Wire Wire Line
	10250 6300 10950 6300
Wire Wire Line
	10250 6200 10950 6200
Wire Wire Line
	10250 6100 10950 6100
Text Label 10950 6400 2    50   ~ 0
jtag_tck
Text Label 10950 6300 2    50   ~ 0
jtag_tdo
Text Label 10950 6200 2    50   ~ 0
jtag_tdi
Text Label 10950 6100 2    50   ~ 0
jtag_tms
Text Label 4300 5000 0    50   ~ 0
SF_SPI_DI
Wire Wire Line
	4300 5000 5000 5000
Text Label 4300 4900 0    50   ~ 0
SF_SPI_DO
Wire Wire Line
	4300 4900 5000 4900
$Comp
L power:GND #PWR?
U 1 1 609FCB57
P 1000 7250
F 0 "#PWR?" H 1000 7000 50  0001 C CNN
F 1 "GND" H 1005 7077 50  0000 C CNN
F 2 "" H 1000 7250 50  0001 C CNN
F 3 "" H 1000 7250 50  0001 C CNN
	1    1000 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A03F0C
P 1650 7250
F 0 "#PWR?" H 1650 7000 50  0001 C CNN
F 1 "GND" H 1655 7077 50  0000 C CNN
F 2 "" H 1650 7250 50  0001 C CNN
F 3 "" H 1650 7250 50  0001 C CNN
	1    1650 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A0B332
P 2250 7250
F 0 "#PWR?" H 2250 7000 50  0001 C CNN
F 1 "GND" H 2255 7077 50  0000 C CNN
F 2 "" H 2250 7250 50  0001 C CNN
F 3 "" H 2250 7250 50  0001 C CNN
	1    2250 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A128A8
P 4150 7450
F 0 "#PWR?" H 4150 7200 50  0001 C CNN
F 1 "GND" H 4155 7277 50  0000 C CNN
F 2 "" H 4150 7450 50  0001 C CNN
F 3 "" H 4150 7450 50  0001 C CNN
	1    4150 7450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A19D3C
P 4850 7450
F 0 "#PWR?" H 4850 7200 50  0001 C CNN
F 1 "GND" H 4855 7277 50  0000 C CNN
F 2 "" H 4850 7450 50  0001 C CNN
F 3 "" H 4850 7450 50  0001 C CNN
	1    4850 7450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A212D5
P 5500 7450
F 0 "#PWR?" H 5500 7200 50  0001 C CNN
F 1 "GND" H 5505 7277 50  0000 C CNN
F 2 "" H 5500 7450 50  0001 C CNN
F 3 "" H 5500 7450 50  0001 C CNN
	1    5500 7450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A28B1A
P 8650 6250
F 0 "#PWR?" H 8650 6000 50  0001 C CNN
F 1 "GND" H 8655 6077 50  0000 C CNN
F 2 "" H 8650 6250 50  0001 C CNN
F 3 "" H 8650 6250 50  0001 C CNN
	1    8650 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A3009C
P 7550 5650
F 0 "#PWR?" H 7550 5400 50  0001 C CNN
F 1 "GND" H 7555 5477 50  0000 C CNN
F 2 "" H 7550 5650 50  0001 C CNN
F 3 "" H 7550 5650 50  0001 C CNN
	1    7550 5650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A3E7D6
P 2600 5800
F 0 "#PWR?" H 2600 5550 50  0001 C CNN
F 1 "GND" H 2605 5627 50  0000 C CNN
F 2 "" H 2600 5800 50  0001 C CNN
F 3 "" H 2600 5800 50  0001 C CNN
	1    2600 5800
	1    0    0    -1  
$EndComp
Text Label 1100 5600 0    50   ~ 0
LED_BLUE
Text Label 1100 5200 0    50   ~ 0
LED_GREEN
Text Label 1100 4800 0    50   ~ 0
LED_RED
Text Notes 2000 5800 0    50   ~ 0
20mA
Wire Wire Line
	2250 5600 2600 5600
Wire Wire Line
	2250 5200 2600 5200
Wire Wire Line
	2250 4800 2600 4800
Wire Wire Line
	1500 5600 1100 5600
Wire Wire Line
	1100 4800 1500 4800
Wire Wire Line
	1500 5200 1100 5200
$Comp
L Device:R_Small R?
U 1 1 61070E19
P 1600 5600
AR Path="/61070E19" Ref="R?"  Part="1" 
AR Path="/6093CD3D/61070E19" Ref="R?"  Part="1" 
AR Path="/6103AB31/61070E19" Ref="R6"  Part="1" 
F 0 "R6" V 1404 5600 50  0000 C CNN
F 1 "R_Small" V 1495 5600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 1600 5600 50  0001 C CNN
F 3 "~" H 1600 5600 50  0001 C CNN
	1    1600 5600
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 5600 1700 5600
$Comp
L Device:R_Small R?
U 1 1 61070E12
P 1600 5200
AR Path="/61070E12" Ref="R?"  Part="1" 
AR Path="/6093CD3D/61070E12" Ref="R?"  Part="1" 
AR Path="/6103AB31/61070E12" Ref="R5"  Part="1" 
F 0 "R5" V 1404 5200 50  0000 C CNN
F 1 "R_Small" V 1495 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 1600 5200 50  0001 C CNN
F 3 "~" H 1600 5200 50  0001 C CNN
	1    1600 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 5200 1700 5200
$Comp
L Device:R_Small R?
U 1 1 61070E0B
P 1600 4800
AR Path="/61070E0B" Ref="R?"  Part="1" 
AR Path="/6093CD3D/61070E0B" Ref="R?"  Part="1" 
AR Path="/6103AB31/61070E0B" Ref="R4"  Part="1" 
F 0 "R4" V 1404 4800 50  0000 C CNN
F 1 "R_Small" V 1495 4800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 1600 4800 50  0001 C CNN
F 3 "~" H 1600 4800 50  0001 C CNN
	1    1600 4800
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 4800 1700 4800
$Comp
L Device:LED D?
U 1 1 61070E04
P 2100 5600
AR Path="/61070E04" Ref="D?"  Part="1" 
AR Path="/6093CD3D/61070E04" Ref="D?"  Part="1" 
AR Path="/6103AB31/61070E04" Ref="D3"  Part="1" 
F 0 "D3" H 2093 5345 50  0000 C CNN
F 1 "LED_BLUE" H 2093 5436 50  0000 C CNN
F 2 "Air_Pollution_Sensor:C503B-GAN-CB0F0791" H 2100 5600 50  0001 C CNN
F 3 "~" H 2100 5600 50  0001 C CNN
	1    2100 5600
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 61070DFE
P 2100 5200
AR Path="/61070DFE" Ref="D?"  Part="1" 
AR Path="/6093CD3D/61070DFE" Ref="D?"  Part="1" 
AR Path="/6103AB31/61070DFE" Ref="D2"  Part="1" 
F 0 "D2" H 2093 4945 50  0000 C CNN
F 1 "LED_GREEN" H 2093 5036 50  0000 C CNN
F 2 "Air_Pollution_Sensor:C503B-GAN-CB0F0791" H 2100 5200 50  0001 C CNN
F 3 "~" H 2100 5200 50  0001 C CNN
	1    2100 5200
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 61070DF8
P 2100 4800
AR Path="/61070DF8" Ref="D?"  Part="1" 
AR Path="/6093CD3D/61070DF8" Ref="D?"  Part="1" 
AR Path="/6103AB31/61070DF8" Ref="D1"  Part="1" 
F 0 "D1" H 2093 4545 50  0000 C CNN
F 1 "LED_RED" H 2093 4636 50  0000 C CNN
F 2 "Air_Pollution_Sensor:C503B-GAN-CB0F0791" H 2100 4800 50  0001 C CNN
F 3 "~" H 2100 4800 50  0001 C CNN
	1    2100 4800
	-1   0    0    1   
$EndComp
Connection ~ 2600 5200
Wire Wire Line
	2600 5200 2600 5600
Wire Wire Line
	2600 4800 2600 5200
Text Label 1100 4200 0    50   ~ 0
LED_BLUE
Text Label 1100 4100 0    50   ~ 0
LED_GREEN
Text Label 1100 4000 0    50   ~ 0
LED_RED
Wire Wire Line
	1100 4000 1800 4000
Wire Wire Line
	1100 4100 1800 4100
Wire Wire Line
	1100 4200 1800 4200
Text Notes 8900 5000 0    50   ~ 0
t GNDPLL should not be connected to the board s ground.
Connection ~ 2250 1300
$Comp
L power:+3.3V #PWR0108
U 1 1 6117862D
P 2250 950
F 0 "#PWR0108" H 2250 800 50  0001 C CNN
F 1 "+3.3V" H 2265 1123 50  0000 C CNN
F 2 "" H 2250 950 50  0001 C CNN
F 3 "" H 2250 950 50  0001 C CNN
	1    2250 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1300 2450 1300
$Comp
L Device:C_Small C?
U 1 1 60B89C7A
P 2450 1550
AR Path="/6093CD3D/60B89C7A" Ref="C?"  Part="1" 
AR Path="/6103AB31/60B89C7A" Ref="C?"  Part="1" 
F 0 "C?" H 2542 1596 50  0000 L CNN
F 1 "100nF" H 2542 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2450 1550 50  0001 C CNN
F 3 "~" H 2450 1550 50  0001 C CNN
	1    2450 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 1650 2450 1800
Wire Wire Line
	2450 1450 2450 1300
$Comp
L Device:C_Small C?
U 1 1 60B91A8E
P 2900 1550
AR Path="/6093CD3D/60B91A8E" Ref="C?"  Part="1" 
AR Path="/6103AB31/60B91A8E" Ref="C?"  Part="1" 
F 0 "C?" H 2992 1596 50  0000 L CNN
F 1 "100nF" H 2992 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2900 1550 50  0001 C CNN
F 3 "~" H 2900 1550 50  0001 C CNN
	1    2900 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 1650 2900 1800
Wire Wire Line
	2900 1450 2900 1300
Wire Wire Line
	2450 1800 2900 1800
Connection ~ 2450 1800
Wire Wire Line
	2450 1300 2900 1300
Connection ~ 2450 1300
Wire Wire Line
	2200 1800 2200 2100
Connection ~ 2200 1800
Wire Wire Line
	2200 1800 2450 1800
$Comp
L Device:C_Small C?
U 1 1 60BED48A
P 4750 2650
AR Path="/6093CD3D/60BED48A" Ref="C?"  Part="1" 
AR Path="/6103AB31/60BED48A" Ref="C?"  Part="1" 
F 0 "C?" H 4842 2696 50  0000 L CNN
F 1 "1uF" H 4842 2605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 4750 2650 50  0001 C CNN
F 3 "~" H 4750 2650 50  0001 C CNN
	1    4750 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 60BED490
P 5250 2650
AR Path="/6093CD3D/60BED490" Ref="C?"  Part="1" 
AR Path="/6103AB31/60BED490" Ref="C?"  Part="1" 
F 0 "C?" H 5342 2696 50  0000 L CNN
F 1 "100nF" H 5342 2605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5250 2650 50  0001 C CNN
F 3 "~" H 5250 2650 50  0001 C CNN
	1    5250 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 2750 4750 2900
Wire Wire Line
	4750 2900 5250 2900
Wire Wire Line
	5250 2750 5250 2900
Connection ~ 5250 2900
Wire Wire Line
	5250 2900 5500 2900
Wire Wire Line
	4750 2550 4750 2400
Wire Wire Line
	4750 2400 5250 2400
Wire Wire Line
	5250 2550 5250 2400
Connection ~ 5250 2400
Wire Wire Line
	5250 2400 5550 2400
Wire Wire Line
	5550 2050 5550 2400
Connection ~ 5550 2400
$Comp
L power:+3.3V #PWR?
U 1 1 60BED4A2
P 5550 2050
F 0 "#PWR?" H 5550 1900 50  0001 C CNN
F 1 "+3.3V" H 5565 2223 50  0000 C CNN
F 2 "" H 5550 2050 50  0001 C CNN
F 3 "" H 5550 2050 50  0001 C CNN
	1    5550 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2400 5750 2400
$Comp
L Device:C_Small C?
U 1 1 60BED4A9
P 5750 2650
AR Path="/6093CD3D/60BED4A9" Ref="C?"  Part="1" 
AR Path="/6103AB31/60BED4A9" Ref="C?"  Part="1" 
F 0 "C?" H 5842 2696 50  0000 L CNN
F 1 "100nF" H 5842 2605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5750 2650 50  0001 C CNN
F 3 "~" H 5750 2650 50  0001 C CNN
	1    5750 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 2750 5750 2900
Wire Wire Line
	5750 2550 5750 2400
$Comp
L Device:C_Small C?
U 1 1 60BED4B1
P 6200 2650
AR Path="/6093CD3D/60BED4B1" Ref="C?"  Part="1" 
AR Path="/6103AB31/60BED4B1" Ref="C?"  Part="1" 
F 0 "C?" H 6292 2696 50  0000 L CNN
F 1 "100nF" H 6292 2605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 6200 2650 50  0001 C CNN
F 3 "~" H 6200 2650 50  0001 C CNN
	1    6200 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2750 6200 2900
Wire Wire Line
	6200 2550 6200 2400
Wire Wire Line
	5750 2900 6200 2900
Connection ~ 5750 2900
Wire Wire Line
	5750 2400 6200 2400
Connection ~ 5750 2400
Wire Wire Line
	5500 2900 5500 3200
Connection ~ 5500 2900
Wire Wire Line
	5500 2900 5750 2900
$EndSCHEMATC
