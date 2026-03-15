/**
 * lid_sensor.cpp — MacBook Lid / Hinge Sensor Reader
 *
 * This program attempts to read MacBook lid position data through two
 * hardware interfaces exposed by macOS:
 *
 *   1. AppleSMC (System Management Controller)
 *      - The SMC manages low-level hardware state on every Mac.
 *      - We read the "MSLD" key which reports the clamshell/lid state:
 *          0 = lid closed (clamshell mode)
 *          1 = lid open
 *      - This is the most reliable method but only gives binary state.
 *
 *   2. SMCMotionSensor (Sudden Motion Sensor / Accelerometer)
 *      - Present on older MacBooks and some newer models.
 *      - Reports a 3-axis gravity vector (x, y, z) relative to the
 *        machine's resting orientation.
 *      - By measuring the tilt of this gravity vector we can estimate
 *        how far the lid/screen is tilted back from vertical.
 *      - This is an APPROXIMATION — it measures the whole-machine tilt,
 *        not the hinge angle directly. It works best when the MacBook
 *        base is on a flat surface.
 *
 * Sensor Discovery:
 *   Both services are discovered by name through IOKit's IOServiceGetMatchingService().
 *   We ask the I/O Registry for services named "AppleSMC" and "SMCMotionSensor".
 *   If a service exists, we open a user-client connection to it and issue
 *   IOConnectCallStructMethod() calls to read data.
 *
 * Limitations:
 *   - macOS does NOT expose a true hinge-angle sensor through any public API.
 *   - The MSLD key gives only open/closed, not a continuous angle.
 *   - The accelerometer measures whole-machine tilt, not lid-vs-base angle.
 *   - On Apple Silicon Macs, SMC access may require elevated privileges.
 *   - The SMCMotionSensor service may not be present on all models.
 *
 * Compile:
 *   clang++ -std=c++17 -O2 -Wall \
 *       -framework IOKit -framework CoreFoundation \
 *       -o lid_sensor lid_sensor.cpp
 *
 * Run:
 *   ./lid_sensor          # may need: sudo ./lid_sensor
 */

#include <IOKit/IOKitLib.h>
#include <CoreFoundation/CoreFoundation.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <mach/mach.h>

// ============================================================================
//  SMC interface definitions
// ============================================================================

// SMC key data types — these match Apple's internal protocol structures.
// The SMC protocol uses fixed-size structs passed through IOConnectCallStructMethod.

#define SMC_CMD_READ_KEY_INFO  9   // get key metadata (type, size)
#define SMC_CMD_READ_BYTES    5    // read key value bytes

// The 4-character SMC key is packed as a big-endian uint32.
typedef uint32_t SMCKeyCode;

// SMC value types — 4-char codes indicating the data format.
// Common types: "ui8 " (uint8), "ui16" (uint16), "flt " (float), "flag" (bool), etc.
typedef char SMCKeyType[5];

// Structure sent to SMC for read operations.
// This matches the kernel-side struct expected by AppleSMC's user-client.
#pragma pack(push, 1)
typedef struct {
    SMCKeyCode  key;
    uint8_t     vers[6];
    uint8_t     pLimitData;
    uint8_t     unused1;
    uint32_t    dataSize;
    uint32_t    dataType;
    uint8_t     dataAttributes;
    uint8_t     unused2[3];
    uint8_t     bytes[32];
} SMCParamStruct;
#pragma pack(pop)

// Kernel function selector for AppleSMC user-client methods.
// Selector 2 (kSMCHandleYPCEvent) is the main read/write entry point.
// Selectors 0 (open) and 1 (close) exist but are not needed — IOServiceOpen/Close
// handle connection lifecycle.
static const uint32_t kSMCHandleYPCEvent  = 2;

// ============================================================================
//  Helper: convert 4-char string to SMC key code
// ============================================================================
static SMCKeyCode smcKeyFromString(const char* str) {
    // SMC keys are stored as big-endian 4-byte codes.
    // "MSLD" -> 0x4D534C44
    return ((uint32_t)str[0] << 24) |
           ((uint32_t)str[1] << 16) |
           ((uint32_t)str[2] <<  8) |
           ((uint32_t)str[3]);
}

// ============================================================================
//  SMC Connection Management
// ============================================================================

/**
 * Opens a connection to the AppleSMC IOService.
 *
 * Discovery process:
 *   1. IOServiceMatching("AppleSMC") creates a matching dictionary for
 *      services registered under the "AppleSMC" class name.
 *   2. IOServiceGetMatchingService() searches the I/O Registry and returns
 *      the first matching service object.
 *   3. IOServiceOpen() creates a user-client connection to the service,
 *      giving us an io_connect_t handle for subsequent calls.
 */
static io_connect_t openSMC() {
    io_service_t svc = IOServiceGetMatchingService(
        kIOMainPortDefault,
        IOServiceMatching("AppleSMC")
    );

    if (svc == IO_OBJECT_NULL) {
        fprintf(stderr, "[SMC] AppleSMC service not found in I/O Registry.\n");
        fprintf(stderr, "      This may happen on very old Macs or in VMs.\n");
        return IO_OBJECT_NULL;
    }

    io_connect_t conn = IO_OBJECT_NULL;
    kern_return_t kr = IOServiceOpen(svc, mach_task_self(), 0, &conn);
    IOObjectRelease(svc);

    if (kr != KERN_SUCCESS) {
        fprintf(stderr, "[SMC] Failed to open AppleSMC user-client (error %d).\n", kr);
        fprintf(stderr, "      Try running with sudo.\n");
        return IO_OBJECT_NULL;
    }

    return conn;
}

/**
 * Reads the value of an SMC key.
 *
 * Protocol:
 *   We fill an SMCParamStruct with the key code and command byte, then call
 *   IOConnectCallStructMethod(kSMCHandleYPCEvent, ...).
 *   The kernel fills in the result bytes and dataSize.
 *
 * Two-step process:
 *   Step 1 — READ_KEY_INFO: Queries the key's metadata (type + size).
 *   Step 2 — READ_BYTES:    Reads the actual value bytes using the size
 *            learned in step 1.
 */
static bool readSMCKey(io_connect_t conn, const char* keyStr, SMCParamStruct* result) {
    SMCParamStruct in;
    SMCParamStruct out;
    size_t outSize;
    kern_return_t kr;

    // --- Step 1: Get key info (type + size) ---
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    in.key = smcKeyFromString(keyStr);
    in.dataSize = sizeof(in.bytes);       // tell SMC max buffer size
    in.bytes[0] = SMC_CMD_READ_KEY_INFO;  // command byte

    outSize = sizeof(out);
    kr = IOConnectCallStructMethod(
        conn, kSMCHandleYPCEvent,
        &in, sizeof(in),
        &out, &outSize
    );
    if (kr != KERN_SUCCESS) {
        return false;
    }

    // --- Step 2: Read the value bytes ---
    memset(&in, 0, sizeof(in));
    in.key = smcKeyFromString(keyStr);
    in.dataSize = out.dataSize;           // use size from key info
    in.dataType = out.dataType;           // use type from key info
    in.bytes[0] = SMC_CMD_READ_BYTES;     // command byte

    outSize = sizeof(out);
    kr = IOConnectCallStructMethod(
        conn, kSMCHandleYPCEvent,
        &in, sizeof(in),
        &out, &outSize
    );
    if (kr != KERN_SUCCESS) {
        return false;
    }

    *result = out;
    return true;
}

// ============================================================================
//  Accelerometer (SMCMotionSensor) Access
// ============================================================================

/**
 * The SMCMotionSensor (Sudden Motion Sensor) exposes a 3-axis accelerometer
 * built into many MacBooks. The specific IOConnect function index and output
 * structure varies by model. The most common layout:
 *
 *   - Function selector 5, output is 40 bytes.
 *   - Bytes [0..1] = X-axis (int16, big-endian)
 *   - Bytes [2..3] = Y-axis (int16, big-endian)
 *   - Bytes [4..5] = Z-axis (int16, big-endian)
 *
 * Values are in a device-specific unit; typically ~256 counts per g.
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelData;

static io_connect_t openMotionSensor() {
    // Discover the motion sensor service in the I/O Registry.
    io_service_t svc = IOServiceGetMatchingService(
        kIOMainPortDefault,
        IOServiceMatching("SMCMotionSensor")
    );

    if (svc == IO_OBJECT_NULL) {
        return IO_OBJECT_NULL;
    }

    io_connect_t conn = IO_OBJECT_NULL;
    kern_return_t kr = IOServiceOpen(svc, mach_task_self(), 0, &conn);
    IOObjectRelease(svc);

    if (kr != KERN_SUCCESS) {
        return IO_OBJECT_NULL;
    }

    return conn;
}

static bool readAccelerometer(io_connect_t conn, AccelData* data) {
    // The motion sensor uses IOConnectCallStructMethod with function index 5.
    // The input is empty (NULL, 0) and the output is a 40-byte record.
    uint8_t outputBuf[40];
    size_t outputSize = sizeof(outputBuf);
    memset(outputBuf, 0, sizeof(outputBuf));

    kern_return_t kr = IOConnectCallStructMethod(
        conn,
        5,              // function selector for "read orientation"
        NULL, 0,        // no input
        outputBuf, &outputSize
    );

    if (kr != KERN_SUCCESS) {
        return false;
    }

    // Extract 16-bit signed values (big-endian on most models).
    data->x = (int16_t)((outputBuf[0] << 8) | outputBuf[1]);
    data->y = (int16_t)((outputBuf[2] << 8) | outputBuf[3]);
    data->z = (int16_t)((outputBuf[4] << 8) | outputBuf[5]);

    return true;
}

/**
 * Estimate tilt angle from accelerometer data.
 *
 * When the MacBook is sitting flat on a table with the lid open:
 *   - Gravity is mostly on the Z-axis.
 *   - As the whole machine tilts (or we approximate the lid tilt from
 *     screen-mounted sensor), the Y component changes.
 *
 * We compute the pitch angle: atan2(y, sqrt(x² + z²)) and then map it
 * to an estimated lid angle. The scale factor (256 counts/g) is approximate
 * and may differ per model.
 *
 * NOTE: This is an APPROXIMATION. It measures the tilt of the component
 * containing the accelerometer, not the literal hinge angle between the
 * lid and base.
 */
static double estimateAngleFromAccel(const AccelData& d) {
    // Normalize to g-units (approximate scale: 256 counts per g)
    const double scale = 256.0;
    double ax = d.x / scale;
    double ay = d.y / scale;
    double az = d.z / scale;

    // Pitch angle from gravity vector.
    // atan2(ay, sqrt(ax² + az²)) gives tilt relative to horizontal.
    double pitchRad = atan2(ay, sqrt(ax * ax + az * az));
    double pitchDeg = pitchRad * (180.0 / M_PI);

    // Map pitch to approximate lid angle:
    //   - Flat (lid at ~90°): pitch ≈ 0°
    //   - Tilted back (lid > 90°): pitch becomes negative
    //   - Tilted forward (lid < 90°): pitch becomes positive
    // We define lid angle = 90 - pitch, so flat = 90°, tilted back = 120°, etc.
    double lidAngle = 90.0 - pitchDeg;

    // Clamp to reasonable range
    if (lidAngle < 0.0) lidAngle = 0.0;
    if (lidAngle > 180.0) lidAngle = 180.0;

    return lidAngle;
}

// ============================================================================
//  Alternative: IOPMCopyClamshellState (high-level clamshell detection)
// ============================================================================

/**
 * As a last resort, we can check the I/O Registry directly for the
 * "AppleClamshellState" property on the "IOPMrootDomain" service.
 * This is reliable on all Mac laptops but only gives open/closed.
 */
static int getClamshellState() {
    io_service_t rootDomain = IOServiceGetMatchingService(
        kIOMainPortDefault,
        IOServiceMatching("IOPMrootDomain")
    );

    if (rootDomain == IO_OBJECT_NULL) {
        return -1;  // unknown
    }

    CFTypeRef prop = IORegistryEntryCreateCFProperty(
        rootDomain,
        CFSTR("AppleClamshellState"),
        kCFAllocatorDefault,
        0
    );
    IOObjectRelease(rootDomain);

    if (prop == NULL) {
        return -1;  // property not found (desktop Mac?)
    }

    int state = -1;
    if (CFGetTypeID(prop) == CFBooleanGetTypeID()) {
        state = CFBooleanGetValue((CFBooleanRef)prop) ? 1 : 0;
        // 1 = clamshell closed, 0 = clamshell open (lid up)
    }
    CFRelease(prop);
    return state;
}

// ============================================================================
//  Main loop
// ============================================================================

int main() {
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║        MacBook Lid / Hinge Sensor Reader                   ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  Discovering available sensors via IOKit ...                ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    // --- Discover and open AppleSMC ---
    printf("[1/3] Looking for AppleSMC service...\n");
    io_connect_t smcConn = openSMC();
    bool hasSMC = (smcConn != IO_OBJECT_NULL);
    if (hasSMC) {
        printf("  ✓ AppleSMC connection established.\n");
    } else {
        printf("  ✗ AppleSMC not available.\n");
    }

    // --- Discover and open SMCMotionSensor (accelerometer) ---
    printf("[2/3] Looking for SMCMotionSensor (accelerometer)...\n");
    io_connect_t accelConn = openMotionSensor();
    bool hasAccel = (accelConn != IO_OBJECT_NULL);
    if (hasAccel) {
        printf("  ✓ SMCMotionSensor connection established.\n");
    } else {
        printf("  ✗ SMCMotionSensor not available.\n");
        printf("      (Common on Apple Silicon Macs — sensor may be absent or restricted.)\n");
    }

    // --- Check IOPMrootDomain clamshell property ---
    printf("[3/3] Checking IOPMrootDomain for AppleClamshellState...\n");
    int clamState = getClamshellState();
    bool hasClamshell = (clamState >= 0);
    if (hasClamshell) {
        printf("  ✓ AppleClamshellState property found.\n");
    } else {
        printf("  ✗ AppleClamshellState not found (desktop Mac?).\n");
    }

    // --- Check if we have any usable data source ---
    if (!hasSMC && !hasAccel && !hasClamshell) {
        fprintf(stderr, "\n[ERROR] No lid/hinge sensors accessible.\n");
        fprintf(stderr, "This may be a desktop Mac, a VM, or SIP is blocking access.\n");
        fprintf(stderr, "Try: sudo ./lid_sensor\n");
        return 1;
    }

    printf("\n── Continuous sensor readout (Ctrl+C to stop) ─────────────────\n\n");

    // Main polling loop: read sensors every second and print status.
    while (true) {
        // Clear line and move cursor for clean updates
        printf("\r\033[K");

        // ---- SMC MSLD Key (Lid State) ----
        if (hasSMC) {
            SMCParamStruct result;
            if (readSMCKey(smcConn, "MSLD", &result)) {
                // MSLD returns 1 byte: 0 = lid closed, 1 = lid open
                uint8_t lidState = result.bytes[0];
                printf("  SMC MSLD Key : Lid is %s  │  ",
                       lidState ? "OPEN ✓" : "CLOSED ✗");
            } else {
                printf("  SMC MSLD Key : [read failed]  │  ");
            }
        }

        // ---- Accelerometer-Based Angle ----
        if (hasAccel) {
            AccelData ad;
            if (readAccelerometer(accelConn, &ad)) {
                double angle = estimateAngleFromAccel(ad);
                printf("Current Lid Angle: %.1f°  ", angle);
                printf("(accel x=%d y=%d z=%d)", ad.x, ad.y, ad.z);
            } else {
                printf("Accelerometer: [read failed]");
            }
        } else if (hasClamshell) {
            // Fallback: just report open/closed from IOPMrootDomain
            int cs = getClamshellState();
            if (cs == 1) {
                printf("Current Lid Angle: ~0° (CLOSED — clamshell detected)");
            } else if (cs == 0) {
                printf("Current Lid Angle: ~110° (OPEN — estimated, no accelerometer)");
            } else {
                printf("Clamshell state: unknown");
            }
        } else if (hasSMC) {
            // If only SMC is available, provide a rough estimate from MSLD
            SMCParamStruct result;
            if (readSMCKey(smcConn, "MSLD", &result) && result.bytes[0]) {
                printf("Current Lid Angle: ~110° (OPEN — estimated, no accelerometer)");
            } else {
                printf("Current Lid Angle: ~0° (CLOSED)");
            }
        }

        printf("\n");
        fflush(stdout);

        // Sleep 1 second between polls
        sleep(1);

        // Move cursor up to overwrite previous line
        printf("\033[1A");
    }

    // Cleanup (unreachable due to while(true), but good practice)
    if (hasSMC) IOServiceClose(smcConn);
    if (hasAccel) IOServiceClose(accelConn);

    return 0;
}
