import argparse
from multiprocessing import Process, Queue, Event, Lock
import socket
import fcntl
import time
import crcmod
import subprocess
import serial
import struct
import random
import select
import queue
import os
import json
import re
from smbus2 import SMBus, i2c_msg

from mfc_gpio_util import gpio_get, gpio_set

HAVE_GPS = True

def set_scheduler(nice):
    os.nice(nice)

def send_data(name, output, data):
    data_string = " ".join("{}:{}".format(key, data[key]) for key in sorted(data))
    output.put(f"{name} {data_string}")

######################################################################
#                   UPTIME
######################################################################

MEMINFO_KEYS = set(("MemTotal", "MemFree", "MemAvailable", "Buffers", "Cached"))

def uptime(name, output, running, lock, nice):
    set_scheduler(nice)
    data = {}
    while running.is_set():
        try:
            with open("/proc/uptime") as f:
                uptime_str = f.read().strip().split()
                data["uptime"], data["idle"] = uptime_str
            
            with open("/proc/meminfo") as f:
                for l in f:
                    s = l.strip().split()
                    if len(s) == 3 and s[2] == "kB" and s[0][:-1] in MEMINFO_KEYS:
                        key = s[0][:-1]
                        data[key.upper()] = s[1]

            send_data(name, output, data)
        except Exception as e:
            output.put(f"{name}_ERROR: Failed to read system info: {str(e)}")
            print(f"[{name}] ERROR: Failed to read system info: {str(e)}")
        time.sleep(15)

######################################################################
#                   BOARDMON
######################################################################

# INA3221 has 3 shunt/bus voltage pairs.
INA3221_PAIRS = [(0x01, 0x02), (0x03, 0x04), (0x05, 0x06)]

def ina3221_voltage(bus_reg):
    return (bus_reg >> 3) * 0.008

def ina3221_current(shunt_reg, shunt_resistance):
    shunt_voltage = (shunt_reg >> 3) * 40e-6
    return shunt_voltage / shunt_resistance

def ina3221(bus, address, *channel_names_and_shunt_resistances, init=False):
    if init:
        return
    data = {}
    for (name, shunt_resistance), (shunt_voltage_reg, bus_voltage_reg) in zip(
        channel_names_and_shunt_resistances, INA3221_PAIRS
    ):
        shunt_write = i2c_msg.write(address, [shunt_voltage_reg])
        shunt_read = i2c_msg.read(address, 2)
        bus_write = i2c_msg.write(address, [bus_voltage_reg])
        bus_read = i2c_msg.read(address, 2)
        bus.i2c_rdwr(shunt_write, shunt_read)
        bus.i2c_rdwr(bus_write, bus_read)
        (shunt_reg,) = struct.unpack(">h", shunt_read.buf[: shunt_read.len])
        (bus_reg,) = struct.unpack(">h", bus_read.buf[: bus_read.len])
        data[f"{name}_V"] = "{:.3f}".format(ina3221_voltage(bus_reg))
        data[f"{name}_A"] = "{:.3f}".format(ina3221_current(shunt_reg, shunt_resistance))
    return data

DEG_C_TO_DEG_K = 273.15

def at30ts75a_temperature(temp_reg):
    temp_milli_deg_c = (temp_reg >> 4) * 62.5
    return (temp_milli_deg_c + DEG_C_TO_DEG_K * 1000) / 1000

def at30ts75a(bus, address, sensor_name, init=False):
    if init:
        conf_write = i2c_msg.write(address, [0x01, 0x60, 0x00])
        bus.i2c_rdwr(conf_write)
        return
    temp_write = i2c_msg.write(address, [0x0])
    temp_read = i2c_msg.read(address, 2)
    bus.i2c_rdwr(temp_write, temp_read)
    (temp_reg,) = struct.unpack(">h", temp_read.buf[: temp_read.len])
    data = {}
    data[sensor_name] = at30ts75a_temperature(temp_reg)
    return data

def ltc2946_voltage(bus_reg):
    return (bus_reg >> 4) * 0.025

def ltc2946_current(shunt_reg, shunt_resistance):
    return (shunt_reg >> 4) * (25e-6 / shunt_resistance)

def ltc2946_power(power_reg, shunt_resistance):
    return power_reg * (6.25e-7 / shunt_resistance)

def ltc2946_adin(adin_reg):
    return (adin_reg >> 4) * 0.5e-3

def ltc2946(bus, address, name, shunt_resistance, adin_name, adin_gain, init=False):
    if init:
        if adin_name is None:
            conf_write = i2c_msg.write(address, [0x0, 0x18])
        else:
            conf_write = i2c_msg.write(address, [0x0, 0x1B])
        bus.i2c_rdwr(conf_write)
        return
    data = {}
    shunt_write = i2c_msg.write(address, [0x14])
    shunt_read = i2c_msg.read(address, 2)
    bus_write = i2c_msg.write(address, [0x1E])
    bus_read = i2c_msg.read(address, 2)
    power_write = i2c_msg.write(address, [0x05])
    power_read = i2c_msg.read(address, 3)
    bus.i2c_rdwr(shunt_write, shunt_read)
    bus.i2c_rdwr(bus_write, bus_read)
    bus.i2c_rdwr(power_write, power_read)
    (shunt_reg,) = struct.unpack(">H", shunt_read.buf[: shunt_read.len])
    (bus_reg,) = struct.unpack(">H", bus_read.buf[: bus_read.len])
    (power_reg,) = struct.unpack(">I", power_read.buf[: power_read.len] + b"\x00")
    power_reg >>= 8
    data[f"{name}_V"] = "{:.3f}".format(ltc2946_voltage(bus_reg))
    data[f"{name}_A"] = "{:.3f}".format(ltc2946_current(shunt_reg, shunt_resistance))
    data[f"{name}_P"] = "{:.3f}".format(ltc2946_power(power_reg, shunt_resistance))
    if adin_name is not None:
        adin_write = i2c_msg.write(address, [0x28])
        adin_read = i2c_msg.read(address, 2)
        bus.i2c_rdwr(adin_write, adin_read)
        (adin_reg,) = struct.unpack(">H", adin_read.buf[: adin_read.len])
        data[adin_name] = "{:.3f}".format(ltc2946_adin(adin_reg) * adin_gain)
    return data

SENSOR_CYCLE_TIME = 1
HEATER_FORCE_ON_TIME = 3
HEATER_CYCLE_TIME = 240

SENSORS = [
    (0x40, ina3221, (("VDD_1V0_AUX", 0.05), ("VDD_2V5_AUX", 0.05), ("VDD_3V3_GPS", 0.05))),
    (0x41, ina3221, (("VDD_3V3_M2M", 0.01), ("VDD_3V3_M2M2", 0.01))),
    (0x48, at30ts75a, ("TEMP0",)),
    (0x49, at30ts75a, ("TEMP1",)),
    (0x4A, at30ts75a, ("TEMP2",)),
    (0x4B, at30ts75a, ("TEMP3",)),
    (0x6A, ltc2946, ("VDD_28V0_TTCFE0", 0.02, "VDD_28V0_TTCFE0_V", 16.8)),
    (0x6B, ltc2946, ("VDD_28V0_TTCFE1", 0.02, "VDD_28V0_TTCFE1_V", 16.8)),
    (0x6C, ltc2946, ("VDD_28V0_HSIO", 0.02, "VDD_5V0_HSIO_V", 3)),
    (0x6D, ltc2946, ("VDD_28V0_MFC", 0.02, "VDD_5V0_V", 3)),
]

def board_heater_enable(enable):
    gpio_set("HEAT_EN", enable)

def board_id():
    board_id = 0
    return board_id

def board_mon(name, output, running, lock, nice):
    set_scheduler(nice)
    bus = SMBus(16)
    board_heater_enable(False)
    heater_timer = time.monotonic()
    def board_heater(data):
        nonlocal heater_timer
        now = time.monotonic()
        time_since_last_force = now - heater_timer
        heater_force_state = time_since_last_force < HEATER_FORCE_ON_TIME
        if time_since_last_force > HEATER_CYCLE_TIME:
            heater_timer += HEATER_CYCLE_TIME
        min_temp = min(float(data["TEMP{}".format(i)]) for i in range(4))
        min_temp -= DEG_C_TO_DEG_K
        heater_enable = (min_temp < 0) or heater_force_state
        board_heater_enable(heater_enable)
        data["HEATER_EN"] = int(heater_enable)
    for sensor, sensor_function, args in SENSORS:
        sensor_function(bus, sensor, *args, init=True)
    heater_timer = time.monotonic()
    prev = time.monotonic()
    try:
        while running.is_set():
            data = {}
            for sensor, sensor_function, args in SENSORS:
                data.update(sensor_function(bus, sensor, *args))
            board_heater(data)
            data["BOARD_ID"] = board_id()
            now = time.monotonic()
            sleep_time = max((prev + SENSOR_CYCLE_TIME) - now, 0.0)
            data["idle"] = "{:.3}".format(sleep_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                prev += SENSOR_CYCLE_TIME
            else:
                prev = now
            data_string = " ".join("{}:{}".format(key, data[key]) for key in sorted(data))
            output.put(f"{name} {data_string}")
    finally:
        board_heater_enable(False)

######################################################################
#                   PWR_SW
######################################################################

POWER_ENABLES = (
    ("HSIO_5V0", "HSIO_5V0_EN"),
    ("TTCFE0", "TTCFE0_DISABLE_L"),
    ("TTCFE1", "TTCFE1_DISABLE_L"),
)
SWITCH_TOGGLE_INTERVAL = 5 * 60

def power_toggle(name, output, running, lock, nice):
    for switch_name, pin_name in POWER_ENABLES:
        gpio_set(pin_name, False)
    switch_state = [False for _ in POWER_ENABLES]
    data = {}
    for switch_name, _ in POWER_ENABLES:
        data[switch_name] = 0
    toggle_index = 0
    try:
        while running.is_set():
            time.sleep(SWITCH_TOGGLE_INTERVAL)
            switch_state[toggle_index] = not switch_state[toggle_index]
            toggle_index = (toggle_index + 1) % len(POWER_ENABLES)
            with lock:
                for (switch_name, pin_name), enabled in zip(
                    POWER_ENABLES, switch_state
                ):
                    gpio_set(pin_name, enabled)
                    data[switch_name] = int(enabled)
            send_data(name, output, data)
    finally:
        with lock:
            for switch_name, pin_name in POWER_ENABLES:
                gpio_set(pin_name, False)

######################################################################
#                   GPS
######################################################################

def set_gps_reset(lock, reset):
    with lock:
        gpio_set("GPS_RESET_L", not reset)

def set_gps_power(lock, power):
    with lock:
        gpio_set("GPS_PWR_EN", power)

def set_osc_power(lock, power):
    with lock:
        gpio_set("HSIO_5V0_EN", power)
        gpio_set("HSIO_MIPI_EN", power)

GPS_COM_PORT = 3
GPS_BOOT_TIMEOUT = 30
GPS_REPORT_INTERVAL = 0.5
GPS_BAUDRATE = 230400
GPS_LISTEN_TIME = 24 * 60 * 60
GPS_COMMAND_INTERVAL = 10

GPS_CRC = crcmod.mkCrcFun(poly=0x104C11DB7, initCrc=0, rev=True, xorOut=0)

HWMONITOR_TYPES = {
    0x01: "TEMP0", 0x02: "ANT_CURR", 0x06: "VDD_3V3", 0x07: "ANT_VOLT",
    0x08: "VDD_1V2", 0x0F: "REG_VOLT", 0x11: "VDD_1V8", 0x15: "VDD_5V0",
    0x16: "TEMP1", 0x17: "VDD_CORE",
}

def gps_crc(data: bytes):
    return GPS_CRC(data)

def add_ascii_crc(s: str):
    b = s.encode("ascii")
    return b"#" + b + "*{:08x}".format(gps_crc(b)).encode("ascii")

def remove_ascii_crc(b: bytes) -> str:
    if not b.startswith(b"#"):
        raise ValueError("Missing prefix?")
    if b[-9:-8] != b"*":
        raise ValueError("Missing suffix?")
    expected_crc = int(b[-8:].decode("ascii"), 16)
    if gps_crc(b[1:-9]) != expected_crc:
        raise ValueError("CRC error!")
    return b[1:-9].decode("ascii")

def gps(name, output, running, lock, nice):
    set_scheduler(nice)
    set_gps_reset(lock, reset=False)
    set_gps_power(lock, power=False)
    set_osc_power(lock, power=False)

    data = {
        "BOOT_TIMEOUTS": 0, "LOGS_RECEIVED": 0, "RX_ERRS": 0, "BYTES_TX": 0,
        "BYTES_RX": 0, "POWER_CYCLES": 0, "RESETS": 0,
    }
    last_report = time.monotonic()

    def check_report():
        nonlocal last_report
        now = time.monotonic()
        if now > last_report + GPS_REPORT_INTERVAL:
            send_data(name, output, data)
            last_report = now

    def ser_read_into(ser, buf, timeout=1.0):
        try:
            ser.timeout = timeout
            b = ser.read(256)
            if not b:
                 print(f"[{name}] WARNING: Serial read timed out or returned no data.")
                 raise serial.SerialException("Read timed out or returned no data.")
            data["BYTES_RX"] += len(b)
            buf.extend(b)
        except serial.SerialException as e:
            output.put(f"GPS_ERROR ser_read_into: {str(e)}")
            print(f"[{name}] ERROR: Serial read failed with exception: {str(e)}")
            raise
    
    def ser_write(ser, buf):
        written = ser.write(buf)
        data["BYTES_TX"] += written
        return written

    def wait_for_gps_ready(ser):
        buf = bytearray()
        expire = time.monotonic() + GPS_BOOT_TIMEOUT
        print(f"[{name}] INFO: Waiting for GPS ready message...")
        while running.is_set():
            check_report()
            try:
                ser_read_into(ser, buf, timeout=1.0)
            except serial.SerialException:
                print(f"[{name}] WARNING: Serial read failed during wait_for_gps_ready. Restarting sequence.")
                return False

            if buf.endswith(f"[COM{GPS_COM_PORT}]".encode()):
                print(f"[{name}] SUCCESS: GPS ready message received!")
                return True
            if time.monotonic() > expire:
                data["BOOT_TIMEOUTS"] += 1
                print(f"[{name}] FAILED: GPS boot timed out.")
                return False

    log_buffer = bytearray()
    def yield_logs(ser, timeout):
        nonlocal log_buffer
        expire = time.monotonic() + timeout
        while running.is_set():
            if time.monotonic() > expire:
                print(f"[{name}] WARNING: Yield logs timed out.")
                raise TimeoutError("Yield logs timed out.")
            
            try:
                ser_read_into(ser, log_buffer, timeout=1.0)
            except serial.SerialException:
                return

            check_report()

            while True:
                start = log_buffer.find(b"#")
                if start == -1: break
                end = log_buffer.find(b"\r\n")
                if end == -1: break
                
                msg = log_buffer[start:end]
                log_buffer = log_buffer[end + 1 :]

                if msg:
                    try:
                        unwrapped_message = remove_ascii_crc(msg)
                        data["LOGS_RECEIVED"] += 1
                        yield unwrapped_message
                        expire = time.monotonic() + timeout
                    except Exception as e:
                        data["RX_ERRS"] += 1
                        output.put(f"GPS_ERROR CRC_ERR:1")
                        output.put(f"GPS_ERROR RX_DETAILS:{str(e)}")
                        print(f"[{name}] WARNING: Corrupted message received, skipping. Error: {str(e)}")
                        return

    def gps_command(ser, command, *args, expect_reply=True):
        print(f"[{name}] INFO: Sending command: {command}...")
        port = "THISPORT"; seq = "0"; idle = "0"; tstatus = "UNKNOWN"; week = "0"
        second = "0.0"; rstatus = "0"; reserved = "0"; version = "0"
        cstr = (f"{command}A,{port},{seq},{idle},{tstatus},{week},{second},{rstatus}"
                + f",{reserved},{version};" + ",".join(args))
        msg = add_ascii_crc(cstr) + b"\n"
        ser_write(ser, msg)

        if expect_reply:
            print(f"[{name}] INFO: Waiting for reply to command...")
            for log in yield_logs(ser, timeout=1.0):
                if log is None:
                    print(f"[{name}] WARNING: No logs received after command. Assuming failure.")
                    return
                if log.split(",")[0] == command + "R":
                    print(f"[{name}] SUCCESS: Reply to {command} received.")
                    return
                else:
                    yield log

    systems = set()
    def process_log(log):
        nonlocal data
        nonlocal systems
        log_name = log.split(",")[0]
        key = "NUM_{}".format(log_name)
        data[key] = data.get(key, 0) + 1
        parts = log.split(";")[1].split(",")
        if log_name == "HWMONITORA":
            num_measurements = int(parts[0])
            assert num_measurements * 2 + 1 == len(parts), (num_measurements, len(parts))
            for reading, status in zip(parts[1::2], parts[2::2]):
                reading = float(reading)
                status = int(status, 16)
                reading_type = status >> 8
                monitor_type = HWMONITOR_TYPES[reading_type]
                if monitor_type.startswith("TEMP"):
                    reading += DEG_C_TO_DEG_K
                data[monitor_type] = "{:.3f}".format(reading)
        elif log_name == "BESTSATSA":
            num_sats = int(parts[0])
            assert num_sats * 4 + 1 == len(parts), (num_sats, len(parts))
            for system, sat_id, status, signal_mask in zip(parts[1::4], parts[2::4], parts[3::4], parts[4::4]):
                systems.add(system)
            for system in systems:
                data["NUM_{}_SATS".format(system)] = 0
            for system, sat_id, status, signal_mask in zip(parts[1::4], parts[2::4], parts[3::4], parts[4::4]):
                data["NUM_{}_SATS".format(system)] += 1
        elif log_name == "BESTXYZA":
            assert len(parts) == 28, len(parts)
            data["HAVE_POS"] = int(parts[0] == "SOL_COMPUTED")
            data["HAVE_VEL"] = int(parts[8] == "SOL_COMPUTED")
            data["NUM_SATS"] = int(parts[20])
            data["NUM_SATS_IN_SOL"] = int(parts[21])
            data["P-X"] = float(parts[2])
            data["P-Y"] = float(parts[3])
            data["P-Z"] = float(parts[4])
            data["PSD-X"] = float(parts[5])
            data["PSD-Y"] = float(parts[6])
            data["PSD-Z"] = float(parts[7])
            data["V-X"] = float(parts[10])
            data["V-Y"] = float(parts[11])
            data["V-Z"] = float(parts[12])
            data["VSD-X"] = float(parts[13])
            data["VSD-Y"] = float(parts[14])
            data["VSD-Z"] = float(parts[15])
        elif log_name == "CLOCKSTEERINGA":
            data["CLK_SOURCE"] = parts[0]
            data["CLK_STATE"] = parts[1]
            data["CLK_PERIOD"] = int(parts[2])
            data["CLK_PULSEWIDTH"] = float(parts[3])
            data["CLK_BW"] = float(parts[4])
            data["CLK_SLOPE"] = float(parts[5])
            data["CLK_OFFSET"] = float(parts[6])
            data["CLK_DRIFTRATE"] = float(parts[7])
        elif log_name == "CLOCKMODELA":
            data["CLK_STATUS"] = parts[0]
            data["CLK_REJECTCOUNT"] = int(parts[1])
            data["CLK_PROPAGATIONTIME"] = float(parts[2])
            data["CLK_UPDATETIME"] = float(parts[3])
            data["CLK_BIAS"] = float(parts[4])
            data["CLK_BIASRATE"] = float(parts[5])
            data["CLK_BIAS_VARIANCE"] = float(parts[7])
            data["CLK_BIAS_COVARIANCE"] = float(parts[8])
            data["CLK_BIASRATE_VARIANCE"] = float(parts[11])
        elif log_name == "VERSIONA":
            data["GPSCARD_MDOEL"] = parts[2]
            data["GPSCARD_PSN"] = parts[3]
            data["GPSCARD_HWVERSION"] = parts[4]
            data["GPSCARD_SWVERSION"] = parts[5]
            data["GPSCARD_BOOTVERSION"] = parts[6]

    while running.is_set():
        try:
            print(f"[{name}] INFO: Starting GPS test cycle.")
            with serial.Serial(port="/dev/ttyLP1", baudrate=GPS_BAUDRATE, write_timeout=1, timeout=0) as ser:
                ser.setRTS(True)
                print(f"[{name}] INFO: Powering on GPS and oscillator...")
                set_gps_reset(lock, reset=False)
                set_gps_power(lock, power=True)
                set_osc_power(lock, power=True)
                data["POWER_CYCLES"] += 1

                if not wait_for_gps_ready(ser):
                    print(f"[{name}] FAILED: GPS did not become ready. Retrying.")
                    continue

                print(f"[{name}] INFO: Performing reset...")
                time.sleep(1)
                set_gps_reset(lock, reset=True)
                time.sleep(1)
                set_gps_reset(lock, reset=False)
                data["RESETS"] += 1

                if not wait_for_gps_ready(ser):
                    print(f"[{name}] FAILED: GPS did not become ready after reset. Retrying.")
                    continue

                log_buffer = bytearray()
                print(f"[{name}] INFO: Requesting VERSIONA log...")
                list(gps_command(ser, "LOG", "THISPORT", "VERSIONA", "ONCE", "0.0", "0.0", "NOHOLD"))
                have_version = False
                for log in yield_logs(ser, timeout=2):
                    if log is None: break
                    have_version = have_version or log.split(",")[0] == "VERSIONA"
                if not have_version:
                    data["BOOT_TIMEOUTS"] += 1
                    print(f"[{name}] FAILED: No VERSIONA log received. Retrying.")
                    continue
                
                print(f"[{name}] SUCCESS: GPS communication is stable. Requesting telemetry...")
                list(gps_command(ser, "SERIALCONFIG", "THISPORT", str(GPS_BAUDRATE), "N", "8", "1", "N", "ON"))
                listen_expire = time.monotonic() + GPS_LISTEN_TIME
                command_expire = time.monotonic() + GPS_COMMAND_INTERVAL
                for log in gps_command(ser, "LOG", "THISPORT", "HWMONITORA", "ONTIME", "1.0", "0.0", "NOHOLD"): process_log(log)
                for log in gps_command(ser, "LOG", "THISPORT", "BESTSATSA", "ONTIME", "1.0", "0.0", "NOHOLD"): process_log(log)
                for log in gps_command(ser, "LOG", "THISPORT", "BESTXYZA", "ONTIME", "1.0", "0.0", "NOHOLD"): process_log(log)
                for log in gps_command(ser, "LOG", "THISPORT", "CLOCKSTEERINGA", "ONTIME", "1.0", "0.0", "NOHOLD"): process_log(log)
                for log in gps_command(ser, "LOG", "THISPORT", "CLOCKMODELA", "ONTIME", "1.0", "0.0", "NOHOLD"): process_log(log)
                list(gps_command(ser, "EXTERNALCLOCK", "USER", "10MHZ", "1.0e-35", "1.0e-35", "1.0e-35"))
                list(gps_command(ser, "CLOCKCALIBRATE", "SET", "4400", "2400", "0.08", "0.01"))
                while running.is_set() and time.monotonic() < listen_expire:
                    for log in yield_logs(ser, timeout=3):
                        if log is None: break
                        process_log(log)
                    if time.monotonic() > command_expire:
                        for log in gps_command(ser, "LOG", "THISPORT", "VERSIONA", "ONCE", "0.0", "0.0", "NOHOLD"):
                            process_log(log)
                        command_expire = time.monotonic() + GPS_COMMAND_INTERVAL
        except serial.SerialException as e:
            output.put(f"GPS_FATAL_SERIAL_ERROR: {str(e)}")
            print(f"[{name}] FATAL ERROR: Serial port exception: {str(e)}")
            time.sleep(5)
        except Exception as e:
            output.put(f"GPS_UNHANDLED_ERROR: {str(e)}")
            print(f"[{name}] UNHANDLED ERROR: {str(e)}")
            time.sleep(5)

######################################################################
#                   MAIN
######################################################################

RUNNER = "RUNNER"
HIGH_PRIORITY = -1
NORMAL_PRIORITY = 0
LOW_PRIORITY = 1

TESTS = {
    "UPTIME": (uptime, NORMAL_PRIORITY),
    "BOARD_MON": (board_mon, NORMAL_PRIORITY),
    "GPS": (gps, NORMAL_PRIORITY),
}

if not HAVE_GPS:
    del TESTS["GPS"]

assert RUNNER not in TESTS, "RUNNING not a valid test name"
PROCESS_REPORT_INTERVAL = 10
PRINTS_PER_LOOP = 5

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--test", type=str, default=None)
    args = parser.parse_args()
    output = Queue()
    lock = Lock()
    running = Event()
    running.set()

    tests = {args.test: TESTS[args.test]} if args.test is not None else TESTS
    
    running_tests = {}
    restart_counts = {}

    set_scheduler(HIGH_PRIORITY)

    for test_name, (test_function, nice) in tests.items():
        running_tests[test_name] = Process(
            target=test_function, args=(test_name, output, running, lock, nice)
        )
        restart_counts[test_name] = 0
        running_tests[test_name].start()

    last_process_report = time.monotonic()
    
    try:
        UDP_IP = "172.16.3.2"
        UDP_PORT = 4242
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as console:
            while True:
                for _ in range(PRINTS_PER_LOOP):
                    try:
                        console.sendto(
                            bytes((output.get(block=True, timeout=1) + "\n\r").encode()),
                            (UDP_IP, UDP_PORT)
                        )
                    except queue.Empty:
                        break
                
                now = time.monotonic()
                if (now - last_process_report) > PROCESS_REPORT_INTERVAL:
                    data = {}
                    for test_name, count in restart_counts.items():
                        data[f"{test_name}_restarts"] = count
                        data[f"{test_name}_alive"] = int(running_tests[test_name].is_alive())
                    runner_string = " ".join(f"{key}:{data[key]}" for key in sorted(data))
                    output.put(f"{RUNNER} {runner_string}")
                    last_process_report = now
                
                for test_name in running_tests:
                    if not running_tests[test_name].is_alive():
                        running_tests[test_name].join()
                        restart_counts[test_name] += 1
                        test_function, nice = tests[test_name]
                        running_tests[test_name] = Process(
                            target=test_function,
                            args=(test_name, output, running, lock, nice),
                        )
                        running_tests[test_name].start()
    except KeyboardInterrupt:
        running.clear()
        for process in running_tests.values():
            process.join()

if __name__ == "__main__":
    main()
