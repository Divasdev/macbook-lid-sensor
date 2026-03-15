# MacBook Lid / Hinge Sensor Reader

Read MacBook lid position data through macOS IOKit hardware interfaces and estimate the lid opening angle.

## ⚠️ Important Limitations

**macOS does NOT expose a continuous hinge-angle sensor through any public API.**

What *is* available:

| Source | Data | Type |
|---|---|---|
| `AppleSMC` → `MSLD` key | Lid open/closed | Binary (0 or 1) |
| `IOPMrootDomain` → `AppleClamshellState` | Clamshell state | Boolean |
| `SMCMotionSensor` | 3-axis accelerometer | Gravity vector (X,Y,Z) |

The accelerometer can estimate **tilt angle** (how far the whole machine is tilted), but this is **not** the same as the hinge angle between the lid and base. It works best when the MacBook base is on a flat surface.

---

## How It Works

### Sensor Discovery

Both programs use **IOKit's I/O Registry** to find hardware services:

1. **`IOServiceMatching("AppleSMC")`** — Creates a matching dictionary for the System Management Controller.
2. **`IOServiceGetMatchingService()`** — Searches the registry and returns a handle to the service.
3. **`IOServiceOpen()`** — Opens a user-client connection for reading data.

### SMC Key Protocol

The SMC stores hardware state as key-value pairs with 4-character keys:

- **`MSLD`** — "MacBook Screen Lid Detect" — `uint8`: 0 = closed, 1 = open
- To read a key: two-step process via `IOConnectCallStructMethod()`:
  1. `READ_KEY_INFO` (cmd 9) → gets key type and data size
  2. `READ_BYTES` (cmd 5) → reads the actual value

### Accelerometer Angle Estimation

The `SMCMotionSensor` returns raw (X, Y, Z) values (~256 counts/g). We compute:

```
pitch = atan2(Y, sqrt(X² + Z²))
lid_angle ≈ 90° - pitch
```

This gives a reasonable approximation when the base sits flat.

---

## Build & Run (C++)

### Prerequisites

- macOS 10.15+ (Catalina or later)
- Xcode Command Line Tools: `xcode-select --install`

### Compile

```bash
make
# or manually:
clang++ -std=c++17 -O2 -Wall \
    -framework IOKit -framework CoreFoundation \
    -o lid_sensor lid_sensor.cpp
```

### Run

```bash
./lid_sensor
# If permission denied:
sudo ./lid_sensor
```

### Expected Output

```
╔══════════════════════════════════════════════════════════════╗
║        MacBook Lid / Hinge Sensor Reader                   ║
╠══════════════════════════════════════════════════════════════╣
║  Discovering available sensors via IOKit ...                ║
╚══════════════════════════════════════════════════════════════╝

[1/3] Looking for AppleSMC service...
  ✓ AppleSMC connection established.
[2/3] Looking for SMCMotionSensor (accelerometer)...
  ✗ SMCMotionSensor not available.
[3/3] Checking IOPMrootDomain for AppleClamshellState...
  ✓ AppleClamshellState property found.

── Continuous sensor readout (Ctrl+C to stop) ─────────────────

  SMC MSLD Key : Lid is OPEN ✓  │  Current Lid Angle: ~110° (OPEN — estimated)
```

---

## Run (Python Fallback)

No compilation needed — requires Python 3.6+.

```bash
python3 lid_sensor.py
# If permission denied:
sudo python3 lid_sensor.py
```

---

## Permissions Notes

| macOS Version | SMC Access | Accelerometer | Clamshell State |
|---|---|---|---|
| 10.15 – 12 | May need sudo | Usually works | Always works |
| 13+ (Ventura) | Needs sudo | May be restricted | Always works |
| Apple Silicon | Needs sudo | Often absent | Always works |

If you get `Failed to open AppleSMC user-client`, run with `sudo`.

---

## Alternative Approaches (True Angle Measurement)

Since macOS doesn't expose the hinge angle, here are alternatives that *can* measure it:

### 1. Camera-Based Estimation
Use the FaceTime camera to detect the viewing angle:
- Track face position relative to the camera and infer screen tilt.
- Libraries: OpenCV + dlib face landmark detection.
- Accuracy: ±5-10° depending on lighting.

### 2. External IMU Sensor
Attach a small IMU (inertial measurement unit) to the lid:
- **BNO055** or **MPU6050** connected via USB or BLE.
- Measures true absolute orientation.
- Accuracy: ±1°.

### 3. Hall-Effect Sensor
The MacBook uses internal hall-effect sensors for lid detection:
- These are not exposed via software APIs.
- With external magnetometer hardware, you could measure the magnet
  positions in the lid and base to compute angle.

### 4. Reverse-Engineering AppleSMC
- Apple may store lid angle data in undocumented SMC keys.
- Tools like `smcFanControl` and the `smc` CLI can enumerate all keys.
- Search for keys with `LC`, `LI`, `LA` prefixes (lid-related).
- **Caution**: Accessing undocumented keys may have side effects.

---

## Files

| File | Description |
|---|---|
| `lid_sensor.cpp` | C++ program using IOKit to access SMC, accelerometer, and clamshell state |
| `lid_sensor.py` | Python fallback using ctypes to call IOKit C functions |
| `Makefile` | Build the C++ program |
| `README.md` | This file |
