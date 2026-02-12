/*
 * ═══════════════════════════════════════════════════════════════════════
 * GPS Rover Core Navigation Library
 * ═══════════════════════════════════════════════════════════════════════
 * 
 * WHAT THIS IS:
 * Core navigation system extracted from working ESP32 GPS Rover.
 * Includes: GPS, Compass, Motor Control, Waypoint Navigation
 * Excludes: Web server (bring your own control protocol)
 * 
 * USE CASE:
 * You want to integrate GPS waypoint navigation into your own system
 * with your own control protocol (Serial, Bluetooth, LoRa, etc.)
 * 
 * FEATURES:
 * ✓ GPS positioning (u-blox)
 * ✓ Compass heading (IST8310) with GPS fallback
 * ✓ Waypoint navigation with alignment before movement
 * ✓ 1 second pause at each waypoint
 * ✓ Three modes: NORMAL, RETURN, LOOP
 * ✓ Motor control (4-wheel drive)
 * 
 * ═══════════════════════════════════════════════════════════════════════
 */

#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include "IST8310.h"

// ══════════════════════════ PIN CONFIGURATION ══════════════════════════

// GPS Serial (UART2)
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Compass I2C
#define I2C_SDA    21
#define I2C_SCL    22

// Motor Driver Pins (4-wheel drive, L298N-style)
#define FL_IN1 13   // Front-Left Forward
#define FL_IN2 12   // Front-Left Backward
#define FR_IN1 27   // Front-Right Forward
#define FR_IN2 26   // Front-Right Backward
#define BL_IN1 4    // Back-Left Forward
#define BL_IN2 25   // Back-Left Backward
#define BR_IN1 18   // Back-Right Forward
#define BR_IN2 19   // Back-Right Backward

// ══════════════════════════ NAVIGATION PARAMETERS ══════════════════════════

// Distance threshold for waypoint detection (meters)
#define WAYPOINT_TOLERANCE    2.5

// Heading error tolerance for moving forward (degrees)
#define HEADING_TOLERANCE     12.0

// Minimum GPS speed for reliable heading (m/s)
#define MIN_GPS_SPEED         0.3

// Minimum satellites for GPS lock
#define MIN_SATELLITES        6

// Maximum waypoints allowed
#define MAX_WAYPOINTS         50

// Update intervals (milliseconds)
#define GPS_UPDATE_INTERVAL        100
#define COMPASS_UPDATE_INTERVAL    100
#define WAYPOINT_PAUSE_MS          1000  // 1 second pause at waypoints

// ══════════════════════════ DATA STRUCTURES ══════════════════════════

struct Waypoint {
  double lat;       // Latitude (decimal degrees)
  double lon;       // Longitude (decimal degrees)
  bool reached;     // Has this waypoint been reached?
};

enum PathMode {
  MODE_NORMAL,   // Origin → WP1 → WP2 → ... → STOP
  MODE_RETURN,   // Origin → WP1 → WP2 → ... → Origin → STOP
  MODE_LOOP      // Origin → WP1 → WP2 → ... → Origin → REPEAT
};

// ══════════════════════════ GLOBAL OBJECTS ══════════════════════════

SFE_UBLOX_GNSS myGNSS;           // GPS module
IST8310 ist8310;                  // Compass module
HardwareSerial gpsSerial(2);      // Hardware serial for GPS

// ══════════════════════════ STATE VARIABLES ══════════════════════════

// GPS State
double currentLat = 0;            // Current latitude
double currentLon = 0;            // Current longitude
double gpsHeading = 0;            // GPS heading (0-360°)
double gpsSpeed = 0;              // GPS speed (m/s)
uint8_t satellites = 0;           // Number of satellites
bool gpsValid = false;            // Is GPS lock valid?

// Compass State
double compassHeading = 0;        // Compass heading (0-360°)
bool compassValid = false;        // Is compass working?
bool compassCalibrated = false;   // Is compass calibrated?
unsigned long lastCompassUpdate = 0;

// Waypoint State
Waypoint waypoints[MAX_WAYPOINTS];  // Waypoint array
int waypointCount = 0;              // Number of waypoints
int currentWP = 0;                  // Current waypoint index
double originLat = 0;               // Origin latitude (first waypoint)
double originLon = 0;               // Origin longitude (first waypoint)

// Navigation State
bool navActive = false;             // Is navigation active?
bool isWaiting = false;             // Is rover waiting at waypoint?
unsigned long waitStartTime = 0;    // When did waiting start?
PathMode pathMode = MODE_NORMAL;    // Current navigation mode
bool returnPointAdded = false;      // Has return point been added?

// Timing
unsigned long lastGPSUpdate = 0;

// ══════════════════════════════════════════════════════════════════════
// MATH & UTILITY FUNCTIONS
// ══════════════════════════════════════════════════════════════════════

/**
 * Normalize angle to 0-360° range
 * @param deg Angle in degrees (any value)
 * @return Normalized angle (0-360°)
 */
double normalizeDegrees(double deg) {
  while (deg < 0) deg += 360;
  while (deg >= 360) deg -= 360;
  return deg;
}

/**
 * Calculate distance between two GPS coordinates using Haversine formula
 * @param lat1, lon1 First coordinate (decimal degrees)
 * @param lat2, lon2 Second coordinate (decimal degrees)
 * @return Distance in meters
 */
double calcDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  
  double a = sin(dLat/2) * sin(dLat/2) + 
             cos(lat1 * PI/180) * cos(lat2 * PI/180) * 
             sin(dLon/2) * sin(dLon/2);
             
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return 6371000.0 * c;  // Earth radius in meters
}

/**
 * Calculate bearing from one coordinate to another
 * @param lat1, lon1 Starting coordinate (decimal degrees)
 * @param lat2, lon2 Target coordinate (decimal degrees)
 * @return Bearing in degrees (0-360°, 0=North, 90=East)
 */
double calcBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = (lon2 - lon1) * PI / 180.0;
  
  double y = sin(dLon) * cos(lat2 * PI / 180.0);
  double x = cos(lat1 * PI/180) * sin(lat2 * PI/180) - 
             sin(lat1 * PI/180) * cos(lat2 * PI/180) * cos(dLon);
             
  return normalizeDegrees(atan2(y, x) * 180.0 / PI);
}

// ══════════════════════════════════════════════════════════════════════
// MOTOR CONTROL FUNCTIONS
// ══════════════════════════════════════════════════════════════════════

/**
 * Initialize motor control pins
 * Sets all 8 motor pins as OUTPUT and LOW (stopped)
 */
void setupMotors() {
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT);
  pinMode(BR_IN1, OUTPUT); pinMode(BR_IN2, OUTPUT);
  
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, LOW);
}

/**
 * Set motor speeds and directions
 * @param left Left side motor speed/direction (-255 to +255)
 * @param right Right side motor speed/direction (-255 to +255)
 * 
 * Positive = Forward, Negative = Backward, Zero = Stop
 * 
 * NOTE: Uses digitalWrite (not analogWrite) for full power
 * This works best with L298N-style motor drivers
 */
void setMotors(int left, int right) {
  // Left motors (Front-Left + Back-Left)
  if (left > 0) {
    // Forward
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
    digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  } else if (left < 0) {
    // Backward
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
    digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
  } else {
    // Stop
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
    digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, LOW);
  }
  
  // Right motors (Front-Right + Back-Right)
  if (right > 0) {
    // Forward
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
  } else if (right < 0) {
    // Backward
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
  } else {
    // Stop
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
    digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, LOW);
  }
}

/**
 * Stop all motors
 */
void stopMotors() {
  setMotors(0, 0);
}

// ══════════════════════════════════════════════════════════════════════
// COMPASS FUNCTIONS
// ══════════════════════════════════════════════════════════════════════

/**
 * Initialize IST8310 compass
 * Returns: true if successful, false if failed
 */
bool setupCompass() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100kHz I2C speed
  delay(100);
  
  if (ist8310.setup(&Wire, NULL)) {
    compassValid = true;
    compassCalibrated = true;  // IST8310 provides calibrated heading
    Serial.println("✓ Compass ready");
    return true;
  } else {
    Serial.println("✗ Compass not found - will use GPS heading");
    return false;
  }
}

/**
 * Update compass heading
 * Should be called regularly (every 100ms recommended)
 */
void updateCompass() {
  if (!compassValid) return;
  
  unsigned long now = millis();
  if (now - lastCompassUpdate < COMPASS_UPDATE_INTERVAL) return;
  lastCompassUpdate = now;
  
  if (ist8310.update()) {
    compassHeading = normalizeDegrees(ist8310.get_heading_degrees());
  }
}

// ══════════════════════════════════════════════════════════════════════
// GPS FUNCTIONS
// ══════════════════════════════════════════════════════════════════════

/**
 * Initialize u-blox GPS module
 * Tries 38400 baud first, falls back to 9600 if needed
 * Returns: true if successful, false if failed
 */
bool setupGPS() {
  // Try 38400 baud first
  gpsSerial.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  if (!myGNSS.begin(gpsSerial)) {
    // If failed, try 9600 and switch to 38400
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    if (myGNSS.begin(gpsSerial)) {
      myGNSS.setSerialRate(38400);
    } else {
      Serial.println("✗ GPS initialization failed");
      return false;
    }
  }
  
  myGNSS.setUART1Output(COM_TYPE_UBX);
  myGNSS.saveConfiguration();
  Serial.println("✓ GPS ready");
  return true;
}

/**
 * Update GPS data
 * Should be called regularly (every 100ms recommended)
 * Updates: currentLat, currentLon, gpsHeading, gpsSpeed, satellites, gpsValid
 */
void updateGPS() {
  if (myGNSS.getPVT()) {
    satellites = myGNSS.getSIV();
    
    // Check if we have valid 3D fix with enough satellites
    if (myGNSS.getFixType() >= 3 && satellites >= MIN_SATELLITES) {
      currentLat = myGNSS.getLatitude() / 10000000.0;
      currentLon = myGNSS.getLongitude() / 10000000.0;
      gpsHeading = myGNSS.getHeading() / 100000.0;
      gpsSpeed = myGNSS.getGroundSpeed() / 1000.0;
      gpsValid = true;
    } else {
      gpsValid = false;
    }
  }
}

// ══════════════════════════════════════════════════════════════════════
// NAVIGATION FUNCTIONS
// ══════════════════════════════════════════════════════════════════════

/**
 * Get current heading from best available source
 * Priority: 1) Compass, 2) GPS (if moving), 3) None
 * 
 * Returns: Heading in degrees (0-360°) or -1 if no heading available
 */
double getCurrentHeading() {
  // PRIORITY 1: Compass (if valid and calibrated)
  if (compassValid && compassCalibrated) {
    return compassHeading;
  }
  
  // PRIORITY 2: GPS heading (if moving fast enough)
  if (gpsValid && gpsSpeed >= MIN_GPS_SPEED) {
    return gpsHeading;
  }
  
  // No valid heading available
  return -1;
}

/**
 * Get heading source name
 * Returns: "COMPASS", "GPS", or "NONE"
 */
String getHeadingSource() {
  if (compassValid && compassCalibrated) return "COMPASS";
  if (gpsValid && gpsSpeed >= MIN_GPS_SPEED) return "GPS";
  return "NONE";
}

/**
 * Main navigation function
 * Should be called regularly (every 100ms recommended)
 * 
 * Navigation logic:
 * 1. Check GPS valid
 * 2. Handle waiting at waypoint (1 second pause)
 * 3. Get current heading (compass or GPS)
 * 4. Calculate distance and bearing to target
 * 5. Check if waypoint reached
 * 6. Calculate heading error
 * 7. Turn in place if error > 12°
 * 8. Move forward if aligned (error ≤ 12°)
 */
void navigate() {
  // Safety check: GPS must be valid
  if (!navActive || !gpsValid) {
    stopMotors();
    return;
  }
  
  // ★ STEP 1: Handle waiting at waypoint (1 second pause)
  if (isWaiting) {
    if (millis() - waitStartTime >= WAYPOINT_PAUSE_MS) {
      isWaiting = false;
      currentWP++;
      
      // Check if all waypoints reached
      if (currentWP >= waypointCount) {
        handlePathComplete();
        return;
      }
      
      Serial.printf("\n→ Moving to WP %d/%d\n", currentWP + 1, waypointCount);
    }
    return;  // Still waiting, don't move
  }
  
  // Check if path complete
  if (currentWP >= waypointCount) {
    handlePathComplete();
    return;
  }
  
  // ★ STEP 2: Get target waypoint
  Waypoint &target = waypoints[currentWP];
  
  // ★ STEP 3: Calculate navigation data
  double distance = calcDistance(currentLat, currentLon, target.lat, target.lon);
  double bearingToTarget = calcBearing(currentLat, currentLon, target.lat, target.lon);
  double currentHeading = getCurrentHeading();
  String headingSource = getHeadingSource();
  
  // ★ STEP 4: Handle no heading (nudge forward to establish GPS heading)
  if (currentHeading < 0) {
    setMotors(60, 60);  // Slow forward
    Serial.println("Getting heading...");
    return;
  }
  
  // ★ STEP 5: Check if waypoint reached
  if (distance < WAYPOINT_TOLERANCE) {
    target.reached = true;
    stopMotors();
    
    Serial.printf("✓ Waypoint %d reached! (%.1fm)\n", currentWP + 1, distance);
    Serial.println("→ Waiting 1 second...");
    
    isWaiting = true;
    waitStartTime = millis();
    return;
  }
  
  // ★ STEP 6: Calculate heading error
  double headingError = bearingToTarget - currentHeading;
  
  // Normalize to [-180, +180] range
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  double absError = fabs(headingError);
  
  // ★ STEP 7: Navigation decision - ALIGN BEFORE MOVING
  
  if (absError > HEADING_TOLERANCE) {
    // NOT ALIGNED - TURN IN PLACE
    if (headingError > 0) {
      setMotors(150, -150);  // Turn right
      Serial.printf("Turn RIGHT %.1f° [%s]\n", absError, headingSource.c_str());
    } else {
      setMotors(-150, 150);   // Turn left
      Serial.printf("Turn LEFT %.1f° [%s]\n", absError, headingSource.c_str());
    }
  } else {
    // ALIGNED - MOVE FORWARD
    setMotors(255, 255);  // Full speed forward
    Serial.printf("→ WP%d %.1fm [%s]\n", currentWP + 1, distance, headingSource.c_str());
  }
}

/**
 * Handle path completion based on current mode
 * Called when all waypoints have been reached
 */
void handlePathComplete() {
  switch (pathMode) {
    case MODE_NORMAL:
      // Stop at last waypoint
      stopMotors();
      navActive = false;
      Serial.println("\n✓ NORMAL mode: Path complete");
      break;
      
    case MODE_RETURN:
      // Return to origin
      if (!returnPointAdded) {
        // First completion - add origin as waypoint
        waypoints[waypointCount].lat = originLat;
        waypoints[waypointCount].lon = originLon;
        waypoints[waypointCount].reached = false;
        waypointCount++;
        returnPointAdded = true;
        
        Serial.println("\n↩ RETURN mode: Returning to origin");
      } else {
        // Second completion (reached origin) - stop
        stopMotors();
        navActive = false;
        waypointCount--;  // Remove return waypoint
        returnPointAdded = false;
        Serial.println("\n✓ RETURN mode: Back at origin");
      }
      break;
      
    case MODE_LOOP:
      // Reset and loop back to start
      currentWP = 0;
      for (int i = 0; i < waypointCount; i++) {
        waypoints[i].reached = false;
      }
      Serial.println("\n🔄 LOOP mode: Restarting from origin");
      break;
  }
}

// ══════════════════════════════════════════════════════════════════════
// PUBLIC API FUNCTIONS (Call these from your control protocol)
// ══════════════════════════════════════════════════════════════════════

/**
 * Add a waypoint to the navigation queue
 * @param lat Latitude (decimal degrees)
 * @param lon Longitude (decimal degrees)
 * @return true if added, false if queue full
 */
bool addWaypoint(double lat, double lon) {
  if (waypointCount >= MAX_WAYPOINTS) {
    Serial.println("✗ Waypoint queue full");
    return false;
  }
  
  waypoints[waypointCount].lat = lat;
  waypoints[waypointCount].lon = lon;
  waypoints[waypointCount].reached = false;
  
  // Save first waypoint as origin
  if (waypointCount == 0) {
    originLat = lat;
    originLon = lon;
  }
  
  waypointCount++;
  Serial.printf("✓ Waypoint %d added: %.6f, %.6f\n", waypointCount, lat, lon);
  return true;
}

/**
 * Clear all waypoints
 */
void clearWaypoints() {
  waypointCount = 0;
  currentWP = 0;
  navActive = false;
  isWaiting = false;
  returnPointAdded = false;
  stopMotors();
  Serial.println("✓ All waypoints cleared");
}

/**
 * Start navigation
 * @return true if started, false if can't start (no GPS or no waypoints)
 */
bool startNavigation() {
  if (waypointCount == 0) {
    Serial.println("✗ No waypoints");
    return false;
  }
  
  if (!gpsValid) {
    Serial.println("✗ GPS not ready");
    return false;
  }
  
  currentWP = 0;
  navActive = true;
  isWaiting = false;
  returnPointAdded = false;
  
  // Reset all waypoints
  for (int i = 0; i < waypointCount; i++) {
    waypoints[i].reached = false;
  }
  
  Serial.println("\n▶ Navigation started");
  return true;
}

/**
 * Stop navigation
 */
void stopNavigation() {
  navActive = false;
  isWaiting = false;
  stopMotors();
  Serial.println("⏹ Navigation stopped");
}

/**
 * Set navigation mode
 * @param mode MODE_NORMAL, MODE_RETURN, or MODE_LOOP
 */
void setNavigationMode(PathMode mode) {
  pathMode = mode;
  Serial.printf("Mode set: %s\n", 
    mode == MODE_NORMAL ? "NORMAL" :
    mode == MODE_RETURN ? "RETURN" : "LOOP");
}

/**
 * Get current navigation status
 * @param status String to store status message
 */
void getNavigationStatus(String &status) {
  if (!gpsValid) {
    status = "GPS: NO FIX";
    return;
  }
  
  if (!navActive) {
    status = "Ready";
    return;
  }
  
  if (isWaiting) {
    status = "✓ WP" + String(currentWP + 1) + " (waiting)";
    return;
  }
  
  if (currentWP < waypointCount) {
    double dist = calcDistance(currentLat, currentLon, 
                               waypoints[currentWP].lat, 
                               waypoints[currentWP].lon);
    status = "→ WP" + String(currentWP + 1) + " (" + String(dist, 1) + "m)";
  } else {
    status = "Path complete";
  }
}

/**
 * Manual motor control (for testing or emergency)
 * @param left Left motor speed (-255 to +255)
 * @param right Right motor speed (-255 to +255)
 */
void manualMotorControl(int left, int right) {
  navActive = false;  // Disable autonomous navigation
  setMotors(left, right);
}

// ══════════════════════════════════════════════════════════════════════
// SETUP AND LOOP
// ══════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n═══════════════════════════════════");
  Serial.println("GPS Rover Core Navigation Library");
  Serial.println("═══════════════════════════════════\n");
  
  // Initialize hardware
  setupMotors();
  setupCompass();
  setupGPS();
  
  Serial.println("\n✓ System ready");
  Serial.println("Add your control protocol here!");
}

void loop() {
  // Update sensors (every 100ms)
  unsigned long now = millis();
  if (now - lastGPSUpdate >= GPS_UPDATE_INTERVAL) {
    updateGPS();
    updateCompass();
    lastGPSUpdate = now;
  }
  
  // Run navigation
  if (navActive) {
    navigate();
  }
  
  // ★ YOUR CONTROL PROTOCOL GOES HERE ★
  // Example: Serial commands, Bluetooth, LoRa, etc.
  // Call the public API functions based on received commands
}
