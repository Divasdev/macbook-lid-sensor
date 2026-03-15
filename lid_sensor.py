#!/usr/bin/env python3
"""
lid_sensor.py — MacBook Lid / Hinge Sensor Reader (Python Fallback)

This script uses ctypes to call macOS IOKit and CoreFoundation C functions,
detecting the lid (clamshell) state and attempting accelerometer-based
angle estimation.

Sensor Discovery:
    We use IOServiceGetMatchingService() from the IOKit framework to find
    services by class name in the macOS I/O Registry:
      - "IOPMrootDomain" → AppleClamshellState property (lid open/closed)
      - "SMCMotionSensor" → Sudden Motion Sensor accelerometer (tilt angle)

Limitations:
    - macOS does NOT expose a continuous hinge-angle sensor publicly.
    - Clamshell detection is binary (open / closed).
    - The accelerometer (if available) measures whole-machine tilt, not the
      hinge angle between lid and base.
    - SMCMotionSensor may not be present on Apple Silicon Macs.
    - Direct AppleSMC access via Python ctypes is fragile and may require sudo.

Run:
    python3 lid_sensor.py
    # or: sudo python3 lid_sensor.py
"""

import ctypes
import ctypes.util
import struct
import sys
import time
import math

# ============================================================================
#  Load macOS frameworks via ctypes
# ============================================================================

def load_framework(name):
    """Load a macOS framework by name using ctypes."""
    path = ctypes.util.find_library(name)
    if path is None:
        raise OSError(f"Cannot find {name} framework")
    return ctypes.CDLL(path)

try:
    iokit = load_framework("IOKit")
    cf = load_framework("CoreFoundation")
except OSError as e:
    print(f"[ERROR] {e}")
    print("This script requires macOS with IOKit framework.")
    sys.exit(1)

# ============================================================================
#  IOKit / CoreFoundation type definitions and function signatures
# ============================================================================

# Common macOS types
kern_return_t = ctypes.c_int
io_object_t = ctypes.c_uint
io_service_t = io_object_t
io_connect_t = io_object_t
mach_port_t = ctypes.c_uint
CFTypeRef = ctypes.c_void_p

# Constants
IO_OBJECT_NULL = 0
KERN_SUCCESS = 0
kIOMainPortDefault = 0  # was kIOMasterPortDefault, renamed in macOS 12+

# --- IOServiceMatching ---
# Creates a matching dictionary for finding IOService objects by class name.
iokit.IOServiceMatching.restype = ctypes.c_void_p
iokit.IOServiceMatching.argtypes = [ctypes.c_char_p]

# --- IOServiceGetMatchingService ---
# Searches the I/O Registry for a service matching the dictionary.
iokit.IOServiceGetMatchingService.restype = io_service_t
iokit.IOServiceGetMatchingService.argtypes = [mach_port_t, ctypes.c_void_p]

# --- IORegistryEntryCreateCFProperty ---
# Reads a property from an IOService entry in the I/O Registry.
iokit.IORegistryEntryCreateCFProperty.restype = CFTypeRef
iokit.IORegistryEntryCreateCFProperty.argtypes = [
    io_service_t,    # entry
    ctypes.c_void_p, # key (CFStringRef)
    ctypes.c_void_p, # allocator
    ctypes.c_uint,   # options
]

# --- IOObjectRelease ---
iokit.IOObjectRelease.restype = kern_return_t
iokit.IOObjectRelease.argtypes = [io_object_t]

# --- IOServiceOpen ---
iokit.IOServiceOpen.restype = kern_return_t
iokit.IOServiceOpen.argtypes = [
    io_service_t,
    ctypes.c_uint,   # owningTask (mach_task_self)
    ctypes.c_uint,   # type
    ctypes.POINTER(io_connect_t),
]

# --- IOServiceClose ---
iokit.IOServiceClose.restype = kern_return_t
iokit.IOServiceClose.argtypes = [io_connect_t]

# --- IOConnectCallStructMethod ---
iokit.IOConnectCallStructMethod.restype = kern_return_t
iokit.IOConnectCallStructMethod.argtypes = [
    io_connect_t,
    ctypes.c_uint,   # selector
    ctypes.c_void_p,  # inputStruct
    ctypes.c_size_t,  # inputStructCnt
    ctypes.c_void_p,  # outputStruct
    ctypes.POINTER(ctypes.c_size_t),  # outputStructCnt
]

# --- CoreFoundation helpers for CFString / CFBoolean ---

# CFStringCreateWithCString
cf.CFStringCreateWithCString.restype = ctypes.c_void_p
cf.CFStringCreateWithCString.argtypes = [
    ctypes.c_void_p,  # allocator
    ctypes.c_char_p,  # cString
    ctypes.c_uint,    # encoding
]

# CFBooleanGetValue
cf.CFBooleanGetValue.restype = ctypes.c_bool
cf.CFBooleanGetValue.argtypes = [ctypes.c_void_p]

# CFRelease
cf.CFRelease.restype = None
cf.CFRelease.argtypes = [ctypes.c_void_p]

# CFGetTypeID
cf.CFGetTypeID.restype = ctypes.c_ulong
cf.CFGetTypeID.argtypes = [ctypes.c_void_p]

# CFBooleanGetTypeID
cf.CFBooleanGetTypeID.restype = ctypes.c_ulong
cf.CFBooleanGetTypeID.argtypes = []

# mach_task_self
libc = ctypes.CDLL(ctypes.util.find_library("System"))
libc.mach_task_self.restype = ctypes.c_uint
libc.mach_task_self.argtypes = []

kCFStringEncodingUTF8 = 0x08000100

# ============================================================================
#  Clamshell (Lid) State Detection
# ============================================================================

def get_clamshell_state():
    """
    Read the AppleClamshellState from IOPMrootDomain.
    
    This queries the I/O Registry for the "IOPMrootDomain" service and reads
    its "AppleClamshellState" property, which is a boolean:
        True  = clamshell closed (lid down)
        False = clamshell open   (lid up)
    
    Returns: True (closed), False (open), or None (unknown / desktop Mac).
    """
    # Find the IOPMrootDomain service
    matching = iokit.IOServiceMatching(b"IOPMrootDomain")
    if matching is None:
        return None
    
    service = iokit.IOServiceGetMatchingService(kIOMainPortDefault, matching)
    if service == IO_OBJECT_NULL:
        return None
    
    # Create a CFString for the property key
    key = cf.CFStringCreateWithCString(None, b"AppleClamshellState", kCFStringEncodingUTF8)
    if key is None:
        iokit.IOObjectRelease(service)
        return None
    
    # Read the property
    prop = iokit.IORegistryEntryCreateCFProperty(service, key, None, 0)
    cf.CFRelease(key)
    iokit.IOObjectRelease(service)
    
    if prop is None:
        return None
    
    # Check if it's a CFBoolean and extract the value
    if cf.CFGetTypeID(prop) == cf.CFBooleanGetTypeID():
        closed = cf.CFBooleanGetValue(prop)
        cf.CFRelease(prop)
        return closed  # True = closed, False = open
    
    cf.CFRelease(prop)
    return None

# ============================================================================
#  Accelerometer (SMCMotionSensor) Access
# ============================================================================

def open_motion_sensor():
    """
    Open a connection to the SMCMotionSensor IOService.
    
    Returns an io_connect_t handle, or None if the sensor is not available.
    The SMCMotionSensor provides the Sudden Motion Sensor / accelerometer
    found in many MacBook models.
    """
    matching = iokit.IOServiceMatching(b"SMCMotionSensor")
    if matching is None:
        return None
    
    service = iokit.IOServiceGetMatchingService(kIOMainPortDefault, matching)
    if service == IO_OBJECT_NULL:
        return None
    
    conn = io_connect_t()
    task = libc.mach_task_self()
    kr = iokit.IOServiceOpen(service, task, 0, ctypes.byref(conn))
    iokit.IOObjectRelease(service)
    
    if kr != KERN_SUCCESS:
        return None
    
    return conn

def read_accelerometer(conn):
    """
    Read accelerometer data from SMCMotionSensor.
    
    Uses IOConnectCallStructMethod with function selector 5.
    Output is a 40-byte buffer; first 6 bytes contain X, Y, Z as
    big-endian int16 values.
    
    Returns (x, y, z) tuple or None on failure.
    """
    output_buf = (ctypes.c_uint8 * 40)()
    output_size = ctypes.c_size_t(40)
    
    kr = iokit.IOConnectCallStructMethod(
        conn,
        5,       # function selector
        None, 0, # no input
        ctypes.byref(output_buf), ctypes.byref(output_size)
    )
    
    if kr != KERN_SUCCESS:
        return None
    
    # Parse big-endian int16 values
    raw = bytes(output_buf)
    x = struct.unpack(">h", raw[0:2])[0]
    y = struct.unpack(">h", raw[2:4])[0]
    z = struct.unpack(">h", raw[4:6])[0]
    
    return (x, y, z)

def estimate_angle(x, y, z):
    """
    Estimate lid tilt angle from accelerometer gravity vector.
    
    Assumes ~256 counts per g. Computes pitch from the Y-axis component
    relative to the total magnitude in the XZ plane, then maps to an
    approximate lid-opening angle.
    
    This is an APPROXIMATION — it measures whole-machine tilt, not the
    literal hinge angle between lid and base.
    """
    scale = 256.0
    ax, ay, az = x / scale, y / scale, z / scale
    
    xz_mag = math.sqrt(ax * ax + az * az)
    if xz_mag < 0.001:
        xz_mag = 0.001  # avoid division by zero
    
    pitch_rad = math.atan2(ay, xz_mag)
    pitch_deg = math.degrees(pitch_rad)
    
    # Map: flat surface → ~90° lid angle, tilted back → >90°
    lid_angle = 90.0 - pitch_deg
    return max(0.0, min(180.0, lid_angle))

# ============================================================================
#  Main
# ============================================================================

def main():
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║      MacBook Lid / Hinge Sensor Reader (Python)            ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  Discovering available sensors via IOKit ...                ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print()

    # --- Discover sensors ---
    
    # 1. Clamshell state (most reliable, works on all MacBook models)
    print("[1/2] Checking IOPMrootDomain for AppleClamshellState...")
    clam = get_clamshell_state()
    has_clamshell = clam is not None
    if has_clamshell:
        print("  ✓ AppleClamshellState property found.")
    else:
        print("  ✗ AppleClamshellState not found (desktop Mac?).")

    # 2. Accelerometer
    print("[2/2] Looking for SMCMotionSensor (accelerometer)...")
    accel_conn = open_motion_sensor()
    has_accel = accel_conn is not None
    if has_accel:
        print("  ✓ SMCMotionSensor connection established.")
    else:
        print("  ✗ SMCMotionSensor not available.")
        print("      (Common on Apple Silicon Macs.)")

    if not has_clamshell and not has_accel:
        print()
        print("[ERROR] No lid sensors accessible.")
        print("This may be a desktop Mac, a VM, or SIP is blocking access.")
        print("Try: sudo python3 lid_sensor.py")
        sys.exit(1)

    print()
    print("── Continuous sensor readout (Ctrl+C to stop) ─────────────────")
    print()

    # --- Limitations notice ---
    if not has_accel:
        print("┌─────────────────────────────────────────────────────────────┐")
        print("│  NOTE: No accelerometer found. Only binary lid state is    │")
        print("│  available. True angle measurement is not possible.        │")
        print("│                                                            │")
        print("│  macOS does NOT expose a hinge-angle sensor publicly.      │")
        print("│  See README.md for alternative approaches:                 │")
        print("│    • Camera-based angle estimation                         │")
        print("│    • External IMU / angle sensor attached via USB/BLE      │")
        print("│    • Hall-effect sensor measurement                        │")
        print("└─────────────────────────────────────────────────────────────┘")
        print()

    try:
        while True:
            parts = []
            
            # Clamshell state
            if has_clamshell:
                state = get_clamshell_state()
                if state is True:
                    parts.append("Lid: CLOSED")
                elif state is False:
                    parts.append("Lid: OPEN ✓")
                else:
                    parts.append("Lid: unknown")
            
            # Accelerometer angle
            if has_accel:
                data = read_accelerometer(accel_conn)
                if data:
                    x, y, z = data
                    angle = estimate_angle(x, y, z)
                    parts.append(f"Current Lid Angle: {angle:.1f}°")
                    parts.append(f"(accel x={x} y={y} z={z})")
                else:
                    parts.append("Accelerometer: [read failed]")
            else:
                # Estimate from binary state
                if has_clamshell:
                    state = get_clamshell_state()
                    if state is True:
                        parts.append("Current Lid Angle: ~0° (CLOSED)")
                    elif state is False:
                        parts.append("Current Lid Angle: ~110° (OPEN — estimated)")
            
            line = "  │  ".join(parts)
            # Overwrite previous line
            sys.stdout.write(f"\r\033[K  {line}")
            sys.stdout.flush()
            
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    finally:
        if has_accel and accel_conn is not None:
            iokit.IOServiceClose(accel_conn)

if __name__ == "__main__":
    main()
