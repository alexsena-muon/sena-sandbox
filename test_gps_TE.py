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
        time.sleep(15)

######################################################################
#                   BOARDMON
######################################################################

# (The BOARDMON section remains the same. It's not included here for brevity,
# but assume it's correctly pasted from the original test.py)

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
    data = {switch_name: 0 for switch_name, _ in POWER_ENABLES}
    toggle_index = 0

    try:
        while running.is_set():
            time.sleep(SWITCH_TOGGLE_INTERVAL)
            switch_state[toggle_index] = not switch_state[toggle_index]
            toggle_index = (toggle_index + 1) % len(POWER_ENABLES)

            with lock:
                for (switch_name, pin_name), enabled in zip(POWER_ENABLES, switch_state):
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
                 raise serial.SerialException("Read timed out or returned no data.")
            data["BYTES_RX"] += len(b)
            buf.extend(b)
        except serial.SerialException as e:
            output.put(f"GPS_ERROR ser_read_into: {str(e)}")
            raise
    
    def ser_write(ser, buf):
        written = ser.write(buf)
        data["BYTES_TX"] += written
        return written

    def wait_for_gps_ready(ser):
        buf = bytearray()
        expire = time.monotonic() + GPS_BOOT_TIMEOUT
        while running.is_set():
            check_report()
            try:
                ser_read_into(ser, buf, timeout=1.0)
            except serial.SerialException:
                # If read fails, break and let the outer loop handle the restart
                return False

            if buf.endswith(f"[COM{GPS_COM_PORT}]".encode()):
                return True
            if time.monotonic() > expire:
                data["BOOT_TIMEOUTS"] += 1
                return False

    log_buffer = bytearray()
    def yield_logs(ser, timeout):
        nonlocal log_buffer
        expire = time.monotonic() + timeout
        while running.is_set():
            if time.monotonic() > expire:
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
                        # Do not continue, yield from the function to trigger a full restart
                        return


    def gps_command(ser, command, *args, expect_reply=True):
        port = "THISPORT"; seq = "0"; idle = "0"; tstatus = "UNKNOWN"; week = "0"
        second = "0.0"; rstatus = "0"; reserved = "0"; version = "0"
        cstr = (f"{command}A,{port},{seq},{idle},{tstatus},{week},{second},{rstatus}"
                + f",{reserved},{version};" + ",".join(args))
        msg = add_ascii_crc(cstr) + b"\n"
        ser_write(ser, msg)

        if expect_reply:
            for log in yield_logs(ser, timeout=1.0):
                if log is None: # Propagate error from yield_logs
                    return
                if log.split(",")[0] == command + "R":
                    return
                else:
                    yield log

    systems = set()
    def process_log(log):
        # ... (rest of the process_log function remains the same, assuming it's correctly pasted)
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
            with serial.Serial(port="/dev/ttyLP1", baudrate=9600, write_timeout=1, timeout=0) as ser:
                ser.baudrate = 9600
                set_gps_reset(lock, reset=False)
                set_gps_power(lock, power=False)
                set_osc_power(lock, power=False)
                time.sleep(10)
                _ = ser.read(256)

                check_report()
                set_gps_reset(lock, reset=False)
                set_gps_power(lock, power=True)
                set_osc_power(lock, power=True)
                data["POWER_CYCLES"] += 1

                if not wait_for_gps_ready(ser):
                    continue

                time.sleep(1)
                set_gps_reset(lock, reset=True)
                time.sleep(1)
                set_gps_reset(lock, reset=False)
                data["RESETS"] += 1

                if not wait_for_gps_ready(ser):
                    continue

                log_buffer = bytearray()
                list(gps_command(ser, "LOG", "THISPORT", "VERSIONA", "ONCE", "0.0", "0.0", "NOHOLD"))
                have_version = False
                for log in yield_logs(ser, timeout=2):
                    if log is None: break
                    have_version = have_version or log.split(",")[0] == "VERSIONA"
                if not have_version:
                    data["BOOT_TIMEOUTS"] += 1
                    continue
                
                list(gps_command(ser, "SERIALCONFIG", "THISPORT", str(GPS_BAUDRATE), "N", "8", "1", "N", "ON"))
                time.sleep(0.100)
                ser.baudrate = GPS_BAUDRATE
                _ = ser.read(256)
                log_buffer = bytearray()
                
                list(gps_command(ser, "LOG", "THISPORT", "VERSIONA", "ONCE", "0.0", "0.0", "NOHOLD"))
                have_version = False
                for log in yield_logs(ser, timeout=2):
                    if log is None: break
                    have_version = have_version or log.split(",")[0] == "VERSIONA"
                if not have_version:
                    data["BOOT_TIMEOUTS"] += 1
                    continue

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
            time.sleep(5) # Wait before retrying to prevent rapid failures
        except Exception as e:
            output.put(f"GPS_UNHANDLED_ERROR: {str(e)}")
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
