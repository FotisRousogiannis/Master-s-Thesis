#!/usr/bin/env python3
"""Segment board v1 — schematic generator.

Single source of truth for the circuit: every component, value and connection is defined
here and written out as a hierarchical KiCad schematic (KiCad 7 file format, opens in KiCad 9).

    python3 build.py            # writes ../segment_board.kicad_sch + sheets + ../lib/segment_board.kicad_sym

Specification: ../SPEC.md · Decisions: docs/decisions/ADR-0013..0015.
Values marked "TBD" depend on measurements still pending (see SPEC.md).
"""

import csv
import datetime
import itertools
from collections import defaultdict
from pathlib import Path

from schematic import Part, Sheet, Writer
from symbols import Library, write_project_library

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
LIBFILE = OUT / "lib" / "segment_board.kicad_sym"

# ---- footprints ----------------------------------------------------------------------------
FP = {
    "R0603": "Resistor_SMD:R_0603_1608Metric",
    "C0603": "Capacitor_SMD:C_0603_1608Metric",
    "C0805": "Capacitor_SMD:C_0805_2012Metric",
    "C1206": "Capacitor_SMD:C_1206_3216Metric",
    "R2512": "Resistor_SMD:R_2512_6332Metric",
    "LED": "LED_SMD:LED_0603_1608Metric",
    "SMA": "Diode_SMD:D_SMA",
    "SMB": "Diode_SMD:D_SMB",
    "CP10": "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
    "L4020": "segment_board:L_4x4mm_SMNR4020",
    "BTN": "Button_Switch_SMD:SW_SPST_TL3342",
    "H1x2": "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
    "H1x3": "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
    "H2x10": "Connector_PinHeader_2.54mm:PinHeader_2x10_P2.54mm_Vertical",
    "MCV2": "Connector_Phoenix_MC:PhoenixContact_MCV_1,5_2-G-3.5_1x02_P3.50mm_Vertical",
    "MCV3": "Connector_Phoenix_MC:PhoenixContact_MCV_1,5_3-G-3.5_1x03_P3.50mm_Vertical",
    "MCV4": "Connector_Phoenix_MC:PhoenixContact_MCV_1,5_4-G-3.5_1x04_P3.50mm_Vertical",
    "MCV5": "Connector_Phoenix_MC:PhoenixContact_MCV_1,5_5-G-3.5_1x05_P3.50mm_Vertical",
    "XT30": "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal",
    "XT30F": "Connector_AMASS:AMASS_XT30UPB-F_1x02_P5.0mm_Vertical",
    "MSTB2": "Connector_Phoenix_MSTB:PhoenixContact_MSTBVA_2,5_2-G-5,08_1x02_P5.08mm_Vertical",
    "FUSE": "Fuse:Fuseholder_Blade_Mini_Keystone_3568",
    "PI": "segment_board:RPi_Socket_2x20_FaceDown",
    "M3": "MountingHole:MountingHole_3.2mm_M3_Pad_Via",
    "M25": "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via",
}

_refs = defaultdict(itertools.count)


def ref(prefix):
    return f"{prefix}{next(_refs[prefix]) + 1}"


# ---- generic parts -------------------------------------------------------------------------
def R(s, value, a, b, fp="R0603", **kw):
    return s.add(Part(ref("R"), "Device:R", value, {"1": a, "2": b}, FP[fp], **kw))


def C(s, value, a, b, fp="C0603", **kw):
    return s.add(Part(ref("C"), "Device:C", value, {"1": a, "2": b}, FP[fp], **kw))


def CP(s, value, plus, minus):
    return s.add(Part(ref("C"), "Device:C_Polarized", value, {"1": plus, "2": minus}, FP["CP10"]))


def LED(s, color, anode, cathode):
    return s.add(Part(ref("D"), "Device:LED", color, {"1": cathode, "2": anode}, FP["LED"]))


def led_indicator(s, net, r, color="green"):
    """net -> resistor -> LED -> GND"""
    mid = f"{net}_LED"
    R(s, r, net, mid)
    LED(s, color, mid, "GND")


def DZ(s, value, cathode, anode, fp="SMB", lcsc=""):
    return s.add(Part(ref("D"), "Device:D_Zener", value, {"1": cathode, "2": anode}, FP[fp], lcsc=lcsc))


def DS(s, value, cathode, anode, lcsc="C8678"):
    return s.add(Part(ref("D"), "Device:D_Schottky", value, {"1": cathode, "2": anode}, FP["SMA"], lcsc=lcsc))


def CONN(s, prefix, pins, fp, value):
    n = len(pins)
    return s.add(Part(ref(prefix), f"Connector_Generic:Conn_01x{n:02d}", value,
                      {str(i + 1): p for i, p in enumerate(pins)}, FP[fp]))


def JUMPER3(s, a, common, b, value):
    """3-pin header + shunt: common net connects to a (pins 1-2) or b (pins 2-3)."""
    return CONN(s, "JP", [a, common, b], "H1x3", value)


def JUMPER2(s, a, b, value):
    return CONN(s, "JP", [a, b], "H1x2", value)


def PWR_FLAG(s, net):
    s.add(Part(ref("#FLG"), "power:PWR_FLAG", "PWR_FLAG", {"1": net}))


def decouple(s, net, *values):
    for v in values:
        C(s, v, net, "GND", fp="C0805" if v in ("10u", "22u") else "C0603")


# =============================================================================================
def power_sheet():
    s = Sheet("power", "power.kicad_sch", "Power: inputs, protection, eFuses, 3.3 V")
    s.text("7.4 V SERVO: XT30 -> 15 A fuse -> P-MOSFET reverse polarity -> TVS -> eFuse TPS259827 (ILIM ~10 A, soft-start, EN) -> bulk caps -> INA219 shunt -> VSERVO. OUT to next board after the TVS (before the eFuse).")
    s.text("5.1 V LOGIC: MSTB -> TVS -> eFuse TPS259472A (ILM ~4.4 A, reverse blocking, OV clamp 5.7 V) -> V5_LOGIC (Pi 5V pins, 3.3 V buck). OUT to next board after the TVS.")
    s.text("E-STOP (board level): NC contact in series with the eFuse EN line; central E-stop cuts the 7.4 V line at the base (ADR-0015)")

    # 7.4 V servo input
    CONN(s, "J", ["VS_RAW", "GND"], "XT30", "XT30 7.4V SERVO IN")
    s.add(Part(ref("F"), "Device:Fuse", "15A blade", {"1": "VS_RAW", "2": "VS_FUSED"}, FP["FUSE"]))
    s.add(Part(ref("Q"), "segment_board:AO4407A", "AO4407A",
               {"5": "VS_FUSED", "6": "VS_FUSED", "7": "VS_FUSED", "8": "VS_FUSED", "4": "QREV_G",
                "1": "VS_PROT", "2": "VS_PROT", "3": "VS_PROT"}, "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", lcsc="C16072",
               note="Reverse-polarity protection: body diode conducts first, then channel enhances (Vgs = -VIN)"))
    R(s, "10k", "QREV_G", "GND")
    DZ(s, "SMBJ9.0A", "VS_PROT", "GND", lcsc="C152088")
    # pass-through to the next board, BEFORE this board's eFuse: the next board limits/switches its own servos
    CONN(s, "J", ["VS_PROT", "GND"], "XT30F", "XT30 7.4V SERVO OUT (next board)")
    s.add(Part(ref("U"), "segment_board:TPS259827", "TPS259827ONRGER",
               {"1": "VS_PROT", "2": "VS_PROT", "3": "VS_PROT", "16": "VS_PROT", "25": "VS_PROT",
                "6": "SERVO_EN_FINAL", "15": "SERVO_DVDT", "7": None, "12": "GND",
                "4": "GND", "5": "GND", "14": "GND", "26": "GND",
                "17": "VS_EF", "18": "VS_EF", "19": "VS_EF", "20": "VS_EF", "21": "VS_EF", "22": "VS_EF",
                "23": "VS_EF", "24": "VS_EF",
                "13": "SERVO_PG", "9": "SERVO_IMON", "8": "SERVO_ILIM", "10": "GND", "11": "GND"},
               "segment_board:TI_RGE0024_2EP", lcsc="C2155761",
               note="RETRY_DLY=GND: latch-off after fault (re-enable via EN). ITIMER open: fastest response."))
    R(s, "150", "SERVO_ILIM", "GND", note="ILIM ~= 1480/R -> ~9.9 A")
    C(s, "3.3n", "SERVO_DVDT", "GND", note="slew ~1.4 V/ms -> inrush ~2.8 A into 2 mF")
    R(s, "1.2k", "SERVO_IMON", "GND", note="IMON 246 uA/A -> 0.295 V/A, 2.95 V at 10 A (to ADS1115 AIN2)")
    R(s, "10k", "SERVO_PG", "+3V3")
    R(s, "100k", "SERVO_EN_FINAL", "GND", note="EN pulled low: servo power OFF by default")
    # board E-stop (normally closed contact) in series with the controller enable line
    R(s, "1k", "SERVO_EN_CTRL", "ESTOP_A")
    CONN(s, "J", ["ESTOP_A", "SERVO_EN_FINAL"], "MCV2", "E-STOP (NC contact)")
    JUMPER2(s, "ESTOP_A", "SERVO_EN_FINAL", "ESTOP bypass")
    CP(s, "1000u 16V", "VS_EF", "GND")
    CP(s, "1000u 16V", "VS_EF", "GND")
    decouple(s, "VS_EF", "10u", "10u", "100n")
    # servo current monitor: INA219 on a 5 mOhm high-side shunt
    R(s, "5m 2W", "VS_EF", "VSERVO", fp="R2512", note="shunt: 8 A -> 40 mV, 0.32 W")
    R(s, "10", "VS_EF", "INA_INP")
    R(s, "10", "VSERVO", "INA_INN")
    C(s, "100n", "INA_INP", "INA_INN")
    s.add(Part(ref("U"), "Sensor_Energy:INA219AxDCN", "INA219AIDCNR",
               {"1": "INA_INP", "2": "INA_INN", "3": "GND", "4": "+3V3", "5": "I2C_SCL", "6": "I2C_SDA",
                "7": "GND", "8": "+3V3"}, "Package_TO_SOT_SMD:SOT-23-8", lcsc="C87469",
               note="A1=VS, A0=GND -> I2C address 0x44"))
    decouple(s, "+3V3", "100n")
    led_indicator(s, "VSERVO", "2.7k")

    # 5.1 V logic input
    CONN(s, "J", ["V5_RAW", "GND"], "MSTB2", "MSTB 5.1V LOGIC IN")
    DZ(s, "SMBJ6.0A", "V5_RAW", "GND")
    CONN(s, "J", ["V5_RAW", "GND"], "MSTB2", "MSTB 5.1V LOGIC OUT (next board)")   # before this board's eFuse
    decouple(s, "V5_RAW", "10u")
    s.add(Part(ref("U"), "segment_board:TPS259472A", "TPS259472ARPWR",
               {"5": "V5_RAW", "1": "V5_UVLO", "2": None, "7": "V5_DVDT", "10": None, "8": "GND",
                "6": "V5_LOGIC", "3": "V5_PG", "4": "V5_PGTH", "9": "V5_ILM"},
               "segment_board:TI_RPW0010A", lcsc="C3662789",
               note="OVCSEL open: output clamp 5.74 V. ITIMER open."))
    R(s, "100k", "V5_RAW", "V5_UVLO")
    R(s, "39k", "V5_UVLO", "GND", note="UVLO ~4.3 V")
    R(s, "750", "V5_ILM", "GND", note="ILIM 4.45 A typ")
    C(s, "3.3n", "V5_DVDT", "GND")
    R(s, "100k", "V5_LOGIC", "V5_PGTH")
    R(s, "47k", "V5_PGTH", "GND")
    R(s, "10k", "V5_PG", "+3V3")
    decouple(s, "V5_LOGIC", "22u", "100n")
    led_indicator(s, "V5_LOGIC", "1.5k")

    # 3.3 V buck, fed from V5_LOGIC or USB VBUS through ideal diodes
    for vin in ("V5_LOGIC", "VBUS_USB"):
        s.add(Part(ref("U"), "segment_board:LM66100", "LM66100DCKR",
                   {"1": vin, "3": "V_BUCK", "2": "GND", "6": "V_BUCK", "5": None},
                   "Package_TO_SOT_SMD:SOT-363_SC-70-6", lcsc="C2869734",
                   note="CE tied to VOUT: ideal-diode OR. ST (open-drain status) unused"))
    decouple(s, "V_BUCK", "10u", "100n")
    s.add(Part(ref("U"), "Regulator_Switching:AP63203WU", "AP63203WU-7",
               {"1": "+3V3", "2": "V_BUCK", "3": "V_BUCK", "4": "GND", "5": "BUCK_SW", "6": "BUCK_BST"},
               "Package_TO_SOT_SMD:TSOT-23-6", lcsc="C780769"))
    C(s, "100n", "BUCK_BST", "BUCK_SW")
    s.add(Part(ref("L"), "Device:L", "2.2u Isat>=3A", {"1": "BUCK_SW", "2": "+3V3"}, FP["L4020"],
               note="L = Vout(Vin-Vout)/(Vin*dIL*fsw) ~ 1.3 uH at 0.8 A ripple; 2.2 uH within datasheet range"))
    decouple(s, "+3V3", "22u", "22u")
    led_indicator(s, "+3V3", "1k")

    for net in ("VS_RAW", "VS_PROT", "VS_EF", "VSERVO", "V5_RAW", "V5_LOGIC", "V_BUCK", "+3V3", "GND", "VBUS_USB"):
        PWR_FLAG(s, net)
    return s


# =============================================================================================
def mcu_sheet():
    s = Sheet("mcu", "mcu.kicad_sch", "ESP32-S3 controller + USB-C")
    s.text("ESP32-S3-WROOM-1-N8R8. Antenna at board edge, no copper under it. IO35-37 used by octal PSRAM; IO0/3/45/46 strapping.")
    io = {
        "1": "GND", "40": "GND", "41": "GND", "2": "+3V3", "3": "ESP_EN", "27": "ESP_IO0",
        "36": "ESP_RXD0", "37": "ESP_TXD0", "13": "USB_DN", "14": "USB_DP",
        "12": "ESP_SDA", "17": "ESP_SCL",                       # IO8, IO9
        "4": "ESP_SERVO_EN", "5": "ESP_PCA_OE",                  # IO4, IO5
        "6": "SERVO_PG", "7": "V5_PG",                           # IO6, IO7
        "8": "ESP_LINK_TX", "9": "ESP_LINK_RX",                  # IO15, IO16
        "25": "STATUS_LED", "24": "ADS_ALRT",                    # IO48, IO47
        "39": "EXP_IO1", "38": "EXP_IO2", "18": "EXP_IO10", "19": "EXP_IO11", "20": "EXP_IO12",
        "21": "EXP_IO13", "22": "EXP_IO14", "10": "EXP_IO17", "11": "EXP_IO18", "23": "EXP_IO21",
        "31": "EXP_IO38", "32": "EXP_IO39", "33": "EXP_IO40", "34": "EXP_IO41", "35": "EXP_IO42",
        "15": None, "16": None, "26": None, "28": None, "29": None, "30": None,   # IO3, IO46, IO45, IO35-37
    }
    s.add(Part(ref("U"), "RF_Module:ESP32-S3-WROOM-1", "ESP32-S3-WROOM-1-N8R8", io,
               "RF_Module:ESP32-S3-WROOM-1", lcsc="C2913201"))
    decouple(s, "+3V3", "22u", "10u", "100n", "100n")
    R(s, "10k", "+3V3", "ESP_EN")
    C(s, "1u", "ESP_EN", "GND")
    s.add(Part(ref("SW"), "Switch:SW_Push", "RESET", {"1": "ESP_EN", "2": "GND"}, FP["BTN"]))
    R(s, "10k", "+3V3", "ESP_IO0")
    s.add(Part(ref("SW"), "Switch:SW_Push", "BOOT", {"1": "ESP_IO0", "2": "GND"}, FP["BTN"]))
    R(s, "1k", "STATUS_LED", "STATUS_LED_A")
    LED(s, "red", "STATUS_LED_A", "GND")   # low Vf: a blue LED (~3 V) stays dark on a 3.3 V GPIO
    # USB-C (device, USB 2.0)
    s.add(Part(ref("J"), "Connector:USB_C_Receptacle_USB2.0_16P", "USB-C",
               {"A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND", "A4": "VBUS_USB", "A9": "VBUS_USB",
                "B4": "VBUS_USB", "B9": "VBUS_USB", "A5": "USB_CC1", "B5": "USB_CC2",
                "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DN", "B7": "USB_DN", "A8": None, "B8": None, "S1": "GND"},
               "segment_board:USB_C_Receptacle_HRO_TYPE-C-31-M-12_S1", lcsc="C165948"))
    R(s, "5.1k", "USB_CC1", "GND")
    R(s, "5.1k", "USB_CC2", "GND")
    s.add(Part(ref("U"), "Power_Protection:USBLC6-2SC6", "USBLC6-2SC6",
               {"1": "USB_DP", "6": "USB_DP", "3": "USB_DN", "4": "USB_DN", "5": "VBUS_USB", "2": "GND"},
               "Package_TO_SOT_SMD:SOT-23-6", lcsc="C7519"))
    # expansion header (spare GPIO)
    exp = ["+3V3", "GND", "EXP_IO1", "EXP_IO2", "EXP_IO10", "EXP_IO11", "EXP_IO12", "EXP_IO13", "EXP_IO14",
           "EXP_IO17", "EXP_IO18", "EXP_IO21", "EXP_IO38", "EXP_IO39", "EXP_IO40", "EXP_IO41", "EXP_IO42",
           "+3V3", "GND", "GND"]
    s.add(Part(ref("J"), "Connector_Generic:Conn_02x10_Odd_Even", "GPIO expansion",
               {str(i + 1): n for i, n in enumerate(exp)}, FP["H2x10"]))
    return s


# =============================================================================================
def i2c_sheet():
    s = Sheet("i2c", "i2c.kicad_sch", "I2C peripherals: master select, PCA9685, TCA9548A, ADS1115")
    s.text("JP: shunt 1-2 = ESP32 is I2C master / control; shunt 2-3 = Raspberry Pi. Never both.")
    s.text("Addresses: PCA9685 0x40, INA219 0x44, ADS1115 0x48, TCA9548A 0x70, AS5600 0x36 behind the mux.")
    JUMPER3(s, "ESP_SDA", "I2C_SDA", "PI_SDA", "I2C SDA master")
    JUMPER3(s, "ESP_SCL", "I2C_SCL", "PI_SCL", "I2C SCL master")
    JUMPER3(s, "ESP_SERVO_EN", "SERVO_EN_CTRL", "PI_SERVO_EN", "SERVO EN source")
    JUMPER3(s, "ESP_PCA_OE", "PCA_OE", "PI_PCA_OE", "PCA OE source")
    R(s, "2.2k", "I2C_SDA", "+3V3")
    R(s, "2.2k", "I2C_SCL", "+3V3")
    R(s, "10k", "PCA_OE", "+3V3", note="outputs disabled until the controller enables them")
    # PCA9685
    pca = {"1": "GND", "2": "GND", "3": "GND", "4": "GND", "5": "GND", "24": "GND", "25": "GND",
           "14": "GND", "28": "+3V3", "23": "PCA_OE", "26": "I2C_SCL", "27": "I2C_SDA"}
    led_pins = ["6", "7", "8", "9", "10", "11", "12", "13", "15", "16", "17", "18", "19", "20", "21", "22"]
    for ch, pin in enumerate(led_pins):
        pca[pin] = f"PWM{ch}"
    s.add(Part(ref("U"), "Driver_LED:PCA9685PW", "PCA9685PW", pca, "Package_SO:TSSOP-28_4.4x9.7mm_P0.65mm", lcsc="C2678753"))
    decouple(s, "+3V3", "10u", "100n")
    for ch in range(9):
        R(s, "220", f"PWM{ch}", f"SERVO{ch + 1}_SIG", note="series protection, servo signal line")
    # TCA9548A
    tca = {"1": "GND", "2": "GND", "21": "GND", "3": "MUX_RST", "12": "GND", "24": "+3V3",
           "22": "I2C_SCL", "23": "I2C_SDA"}
    ch_pins = [("4", "5"), ("6", "7"), ("8", "9"), ("10", "11"), ("13", "14"), ("15", "16"), ("17", "18"), ("19", "20")]
    for ch, (sd, sc) in enumerate(ch_pins):
        name = f"ENC{ch + 1}" if ch < 6 else f"MUX{ch}"
        tca[sd], tca[sc] = f"{name}_SDA", f"{name}_SCL"
    s.add(Part(ref("U"), "Interface_Expansion:TCA9548APWR", "TCA9548APWR", tca, "Package_SO:TSSOP-24_4.4x7.8mm_P0.65mm", lcsc="C130026"))
    decouple(s, "+3V3", "100n")
    R(s, "10k", "MUX_RST", "+3V3")
    for ch in range(8):
        name = f"ENC{ch + 1}" if ch < 6 else f"MUX{ch}"
        R(s, "4.7k", f"{name}_SDA", "+3V3")
        R(s, "4.7k", f"{name}_SCL", "+3V3")
    # ADS1115
    s.add(Part(ref("U"), "Analog_ADC:ADS1115IDGS", "ADS1115IDGSR",
               {"1": "GND", "2": "ADS_ALRT", "3": "GND", "4": "TOUCH1", "5": "TOUCH2", "6": "SERVO_IMON",
                "7": "+3V3", "8": "+3V3", "9": "I2C_SDA", "10": "I2C_SCL"},
               "Package_SO:VSSOP-10_3x3mm_P0.5mm", lcsc="C37593",
               note="AIN0/1 touch sensors, AIN2 servo current (eFuse IMON), AIN3 = 3.3 V rail for ratiometric correction"))
    decouple(s, "+3V3", "100n")
    R(s, "10k", "ADS_ALRT", "+3V3")
    for t in ("TOUCH1", "TOUCH2"):
        R(s, "6.8k", t, "GND", note="ZD10 divider: 17k -> 0.94 V, 2k -> 2.55 V")
        C(s, "100n", t, "GND")
    # expansion: PCA channels 9-15, mux channels 6-7
    exp = ["PWM9", "PWM10", "PWM11", "PWM12", "PWM13", "PWM14", "PWM15", "GND",
           "MUX6_SDA", "MUX6_SCL", "MUX7_SDA", "MUX7_SCL", "I2C_SDA", "I2C_SCL", "+3V3", "GND",
           "VSERVO", "GND", "+3V3", "GND"]
    s.add(Part(ref("J"), "Connector_Generic:Conn_02x10_Odd_Even", "Servo/I2C expansion",
               {str(i + 1): n for i, n in enumerate(exp)}, FP["H2x10"]))
    return s


# =============================================================================================
def connectors_sheet():
    s = Sheet("connectors", "connectors.kicad_sch", "Arm connectors + hardware connection LEDs")
    s.text("Connection LED = current detection on each connector's ground return (no extra pins, no software).")
    s.text("Comparator LM339 (open collector): output low (LED on) when the sense voltage exceeds the reference.")
    s.text("SERVO sense 10 R + SS54, VREF ~10 mV -> detect at 0.5-1.5 mA (servo idle 4-20 mA typ.; confirm by measurement).")
    # references
    R(s, "330k", "+3V3", "VREF_SERVO")
    R(s, "1k", "VREF_SERVO", "GND", note="~10 mV -> ~1 mA servo detect threshold")
    R(s, "100k", "+3V3", "VREF_ENC")
    R(s, "1k", "VREF_ENC", "GND", note="~33 mV -> ~3.3 mA encoder detect threshold (AS5600 ~6.5 mA)")
    R(s, "100k", "+3V3", "VREF_TOUCH")
    R(s, "10k", "VREF_TOUCH", "GND", note="0.3 V: touch sensor present when divider > 0.3 V")

    detectors = []   # (sense net, reference net, led name)
    for n in range(1, 10):
        CONN(s, "J", [f"SERVO{n}_GND", "VSERVO", f"SERVO{n}_SIG"], "MCV3", f"SERVO {n}")
        R(s, "10", f"SERVO{n}_GND", "GND", note="servo current sense: detect > ~1.5 mA, SS54 (5 A) clamps run/stall current")
        DS(s, "SS54", "GND", f"SERVO{n}_GND", lcsc="C22452")
        detectors.append((f"SERVO{n}_GND", "VREF_SERVO", f"SERVO{n}"))
    for n in range(1, 7):
        CONN(s, "J", [f"ENC{n}_GND", "+3V3", f"ENC{n}_SCL", f"ENC{n}_SDA"], "MCV4", f"ENCODER {n}")
        R(s, "10", f"ENC{n}_GND", "GND", note="encoder current sense: 6.5 mA -> 65 mV")
        detectors.append((f"ENC{n}_GND", "VREF_ENC", f"ENC{n}"))
    for n in (1, 2):
        CONN(s, "J", ["+3V3", f"TOUCH{n}"], "MCV2", f"TOUCH {n}")
        detectors.append((f"TOUCH{n}", "VREF_TOUCH", f"TOUCH{n}"))

    # comparators: 5 x LM339 = 20 units, 17 used
    units = [("2", "4", "5"), ("1", "6", "7"), ("13", "10", "11"), ("14", "8", "9")]   # per symbol unit: (out, -, +)
    for chip in range(5):
        cref = ref("U")
        for u in range(4):
            i = chip * 4 + u
            out_pin, minus, plus = units[u]
            if i < len(detectors):
                sense, vref, name = detectors[i]
                nets = {out_pin: f"LED_{name}_K", minus: sense, plus: vref}
                R(s, "1k", "+3V3", f"LED_{name}_A")
                LED(s, "green", f"LED_{name}_A", f"LED_{name}_K")
            else:
                nets = {out_pin: None, minus: "VREF_ENC", plus: "GND"}   # unused: output held off
            s.add(Part(cref, "Comparator:LM339", "LM339DR", nets, "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm",
                       lcsc="C7948", unit=u + 1))
        s.add(Part(cref, "Comparator:LM339", "LM339DR", {"3": "+3V3", "12": "GND"},
                   "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm", lcsc="C7948", unit=5))
        decouple(s, "+3V3", "100n")
    return s


# =============================================================================================
def pi_link_sheet():
    s = Sheet("pi_link", "pi_link.kicad_sch", "Raspberry Pi header + RS-485 LINK")
    s.text("Pi mounts upside down on top: footprint must be MIRRORED relative to a standard HAT (see SPEC R3).")
    s.text("Pi powered from its 5V pins through the 5.1 V eFuse. PI_3V3 is the Pi's own 3.3 V (LED only, never tied to +3V3).")
    pi = {str(n): None for n in range(1, 41)}
    pi.update({"1": "PI_3V3", "17": "PI_3V3", "2": "V5_LOGIC", "4": "V5_LOGIC",
               "6": "GND", "9": "GND", "14": "GND", "20": "GND", "25": "GND", "30": "GND", "34": "GND", "39": "GND",
               "3": "PI_SDA", "5": "PI_SCL", "8": "PI_TXD0", "10": "PI_RXD0", "11": "PI_ESP_EN", "13": "PI_ESP_IO0",
               "15": "PI_SERVO_EN", "16": "PI_PCA_OE", "32": "PI_LINK_TX", "33": "PI_LINK_RX"})
    s.add(Part(ref("J"), "Connector_Generic:Conn_02x20_Odd_Even", "Raspberry Pi GPIO (tall female)", pi, FP["PI"]))
    led_indicator(s, "PI_3V3", "1k", "yellow")
    # Pi <-> ESP32 of this board (UART0 + reset/boot control), enabled by jumpers
    JUMPER2(s, "PI_TXD0", "PI_TXD0_J", "Pi->ESP RX")
    R(s, "470", "PI_TXD0_J", "ESP_RXD0")
    JUMPER2(s, "PI_RXD0", "PI_RXD0_J", "ESP->Pi TX")
    R(s, "470", "PI_RXD0_J", "ESP_TXD0")
    JUMPER2(s, "PI_ESP_EN", "PI_ESP_EN_J", "Pi->ESP EN")
    R(s, "1k", "PI_ESP_EN_J", "ESP_EN")
    JUMPER2(s, "PI_ESP_IO0", "PI_ESP_IO0_J", "Pi->ESP BOOT")
    R(s, "1k", "PI_ESP_IO0_J", "ESP_IO0")
    # RS-485 full duplex
    JUMPER3(s, "ESP_LINK_TX", "LINK_DI", "PI_LINK_TX", "LINK TX source")
    JUMPER3(s, "ESP_LINK_RX", "LINK_RO", "PI_LINK_RX", "LINK RX destination")
    s.add(Part(ref("U"), "Interface_UART:MAX3490xSA", "THVD1451DRBR",
               {"1": "+3V3", "2": "LINK_RO", "3": "LINK_DI", "4": "GND", "5": "LINK_Y", "6": "LINK_Z",
                "7": "LINK_B", "8": "LINK_A"},
               "Package_SON:HVSON-8-1EP_3x3mm_P0.65mm_EP1.6x2.4mm", lcsc="C2671344",
               note="THVD1451: full-duplex RS-485, same pinout as MAX3490 (SO-8). Thermal pad to GND."))
    decouple(s, "+3V3", "100n")
    JUMPER2(s, "LINK_A", "LINK_TERM", "LINK termination")
    R(s, "120", "LINK_TERM", "LINK_B")
    CONN(s, "J", ["LINK_Y", "LINK_Z", "LINK_A", "LINK_B", "GND"], "MCV5", "LINK RS-485 (Y Z A B GND)")
    # mechanics
    for i in range(4):
        s.add(Part(ref("H"), "Mechanical:MountingHole", "M3", {}, FP["M3"]))
    for i in range(4):
        s.add(Part(ref("H"), "Mechanical:MountingHole", "M2.5 Pi", {}, FP["M25"]))
    return s


def apply_passive_lcsc(sheets):
    """LCSC codes for generic parts (R, C, LED, ...) from bom/passives.csv (bom/resolve_passives.py)."""
    table = {}
    path = OUT / "bom" / "passives.csv"
    if path.exists():
        for row in csv.DictReader(open(path, encoding="utf-8")):
            if row["lcsc"]:
                table[(row["value"], row["footprint"])] = row["lcsc"]
    for s in sheets:
        for p in s.parts:
            if not p.lcsc and p.footprint:
                p.lcsc = table.get((p.value, p.footprint.split(":")[1]), "")


def main():
    LIBFILE.parent.mkdir(exist_ok=True)
    write_project_library(LIBFILE)
    lib = Library(LIBFILE)
    sheets = [power_sheet(), mcu_sheet(), i2c_sheet(), connectors_sheet(), pi_link_sheet()]
    apply_passive_lcsc(sheets)
    writer = Writer(lib, "Robotic arm segment board v1", "v1", datetime.date.today().isoformat())
    writer.write(sheets, OUT)
    n = sum(len({(p.ref) for p in s.parts}) for s in sheets)
    print(f"wrote {len(sheets)} sheets, {n} components")


if __name__ == "__main__":
    main()
