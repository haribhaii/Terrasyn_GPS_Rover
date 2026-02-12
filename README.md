ESP32 GPS Rover Project - Chat Summary & Handover
Project Overview
Autonomous GPS rover using ESP32, u-blox GPS, and IST8310 compass for waypoint navigation with web interface control.
Critical Requirements (User's Specifications)

Bearing: Use COMPASS (priority) → GPS (fallback)
Waypoint pause: 1 second stop at each waypoint
Alignment: Rover must align bearing BEFORE moving forward (straight paths, no curves)
Three modes only:

NORMAL: Origin → WP1 → WP2 → WP3 → STOP
RETURN: Origin → WP1 → WP2 → WP3 → Origin → STOP
LOOP: Origin → WP1 → WP2 → WP3 → Origin → REPEAT



Hardware Configuration

ESP32: Main controller
GPS: u-blox NEO-6M/7M (GPIO 16 RX, 17 TX, 38400 baud)
Compass: IST8310 (I2C: SDA=21, SCL=22, 100kHz)
Motors: 4-wheel drive via L298N driver (8 GPIO pins: 13,12,27,26,4,25,18,19)
WiFi: "iTNT Office" / "itntadmin1$" (or AP mode "RoverAP")

Key Technical Decisions
1. Motor Control - digitalWrite (NOT analogWrite)
CRITICAL: User's motors require digital HIGH/LOW, not PWM
cppdigitalWrite(FL_IN1, HIGH);  // ✅ FULL SPEED
analogWrite(FL_IN1, 255);    // ❌ SLOW (doesn't work well)
Reason: L298N-style drivers work best with digital signals for direction pins
2. Compass Issues Fixed
Problem: Compass showed wrong degrees compared to phone
Root Cause: Code was applying calibration offset on top of IST8310's already-calibrated output
Fix: Removed offset, IST8310 library handles calibration internally
cpp// BEFORE (WRONG):
compassHeading = raw + compassCalibrationOffset;

// AFTER (CORRECT):
compassHeading = normalizeDegrees(ist8310.get_heading_degrees());
```

### 3. Speed Control Removed
**User requirement**: "I need my rover to move in only speed which is max speed"
**Implementation**: All speed variation removed, always full throttle (255 or digitalWrite HIGH)

### 4. Code Simplification
**Original**: 1,515 lines with complex enums, boundaries, calibration
**Final v8.0**: 545 lines (-64%), removed:
- Boundary system
- 10 navigation state enums → simple bools
- Heading averaging
- Complex calibration
- Progressive speed reduction

## Final Code Files

### Primary File: `magneto_rover_FINAL_v8.ino`
**Status**: ✅ TESTED with single waypoint, works correctly
**Size**: 545 lines
**Features**: All 4 requirements met, clean code

### Documentation: `ESP32_Rover_COMPLETE_DOCUMENTATION.md`
**Size**: 500+ lines
**Contents**: 
- Complete hardware setup
- Every function explained line-by-line
- Pin configurations
- Troubleshooting guide
- Performance tips

## Code Structure
```
setup()
├── setupMotors()       // Initialize GPIO pins
├── setupCompass()      // IST8310 I2C
├── setupGPS()          // u-blox serial
├── setupWiFi()         // Connect or AP mode
└── setupWebServer()    // HTTP endpoints

loop() (every 100ms)
├── server.handleClient()
├── updateGPS()
├── updateCompass()
└── if (navActive) navigate()
Navigation Logic (Core Algorithm)
cppnavigate() {
  1. Check GPS valid
  2. If waiting → countdown 1 second → advance waypoint
  3. Get heading (COMPASS or GPS)
  4. Calculate distance & bearing to target
  5. If distance < 2.5m → waypoint reached, start 1 sec wait
  6. Calculate heading error = bearingToTarget - currentHeading
  7. If error > 12° → TURN IN PLACE (setMotors(150, -150))
  8. If error ≤ 12° → MOVE FORWARD (setMotors(255, 255))
}
```

**Key**: Step 7-8 ensures alignment before forward movement (Requirement 3)

## Web Server Endpoints
```
GET  /              → HTML interface
GET  /status        → JSON (GPS, heading, waypoints)
GET  /add?lat=X&lon=Y  → Add waypoint
GET  /clear         → Remove all waypoints
GET  /start         → Begin navigation
GET  /stop          → Stop navigation
GET  /move?d=f      → Manual control (f/b/l/r/s)
GET  /mode?m=0      → Set mode (0=NORMAL, 1=RETURN, 2=LOOP)
Critical Parameters
cpp#define WAYPOINT_TOLERANCE  2.5   // meters - when waypoint is "reached"
#define HEADING_TOLERANCE   12.0  // degrees - must align within this to move
#define WAYPOINT_PAUSE_MS   1000  // ms - 1 second wait at waypoints
#define MIN_GPS_SPEED       0.3   // m/s - min speed for GPS heading
#define COMPASS_UPDATE_INTERVAL 100  // ms - 10x faster than typical
Known Issues & Solutions
Issue 1: Compass Not Found

Check I2C wiring (SDA=21, SCL=22)
IST8310 requires 3.3V (NOT 5V)
Code will fallback to GPS heading if compass fails

Issue 2: GPS No Fix

Needs clear sky view (outdoors)
Wait 1-5 minutes for cold start
Requires 6+ satellites

Issue 3: Motors Not Moving

Check battery voltage (7-12V)
Verify 8 GPIO connections
Test with digitalWrite(HIGH) in setup()

Issue 4: Web Interface Not Loading

Check Serial Monitor for IP address
If WiFi fails, connects to "RoverAP" at 192.168.4.1
Ping IP to verify network connection

Testing Checklist
✅ Single Waypoint (User confirmed working)

Add 1 waypoint
Click START
Rover navigates successfully

⏳ Multiple Waypoints (Needs testing)

Add 3 waypoints
Verify 1 second pause at each
Verify straight-line paths (aligns before moving)

⏳ RETURN Mode (Needs testing)

Add 3 waypoints
Select "Return" mode
Verify returns to origin

⏳ LOOP Mode (Needs testing)

Add 2-3 waypoints
Select "Loop" mode
Verify continuous looping

Important Notes for Next Session

User confirmed: Single waypoint works correctly
Need to verify: Multi-waypoint, 1-sec pause, RETURN mode, LOOP mode
Compass heading: Should show [COMPASS] in status, not [GPS]
Motor speed: Always full throttle (digitalWrite HIGH), no PWM
Code version: v8.0 FINAL is the working version

Files to Carry Forward
Essential Files:

✅ magneto_rover_FINAL_v8.ino - Main code (545 lines, tested)
✅ ESP32_Rover_COMPLETE_DOCUMENTATION.md - Complete documentation

Reference Files (if needed):

v8_CLEAN_CHANGES.md - What was changed from v7 to v8
MOTOR_FIX_EXPLANATION.md - Why digitalWrite vs analogWrite
COMPASS_FIX_EXPLANATION.md - Compass calibration fix details
SIMPLIFICATION_SUMMARY.md - Code simplification details

Quick Start for New Session
cpp// Upload: magneto_rover_FINAL_v8.ino
// Check: Serial Monitor at 115200 baud
// Expected output:
✓ Compass ready
✓ GPS ready
✓ WiFi: 192.168.X.X
✓ Ready

// Access web interface at displayed IP
// Test sequence:
1. Wait for GPS lock (6+ satellites)
2. Click map to add 3 waypoints
3. Select mode (NORMAL/RETURN/LOOP)
4. Click START
5. Verify behavior matches requirements
User's WiFi Credentials
cppconst char* WIFI_SSID     = "iTNT Office";
const char* WIFI_PASSWORD = "itntadmin1$";
Context for AI Assistant
User is: Building autonomous GPS rover for outdoor navigation
Technical level: Moderate (understands Arduino, asks good questions)
Focus: Wants simple, working code without unnecessary complexity
Priorities: Reliability > Features, Simplicity > Optimization
Previous issue: Had overly complex code (v7.0, 1515 lines) that was simplified to v8.0 (545 lines)
Testing status: Single waypoint verified working, multi-waypoint pending
