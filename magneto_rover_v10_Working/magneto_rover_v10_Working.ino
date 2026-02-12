/*
 * ═══════════════════════════════════════════════════════════════════════
 * ESP32 GPS Rover v8.0 - CLEAN & OPTIMIZED
 * ═══════════════════════════════════════════════════════════════════════
 * 
 * REQUIREMENTS MET:
 * ✓ Bearing based on COMPASS (preferred) or GPS (fallback)
 * ✓ Rover STOPS at each waypoint (1 second pause)
 * ✓ Rover ALIGNS bearing before moving to next waypoint
 * ✓ Rover moves FORWARD from point to point
 * ✓ THREE MODES ONLY:
 *   - NORMAL: Origin → WP1 → WP2 → WP3 → STOP
 *   - RETURN: Origin → WP1 → WP2 → WP3 → Origin → STOP
 *   - LOOP:   Origin → WP1 → WP2 → WP3 → Origin → REPEAT
 * 
 * Version: 8.0
 * Date: February 2026
 * ═══════════════════════════════════════════════════════════════════════
 */

#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include "IST8310.h"

// ══════════════════════════ CONFIGURATION ══════════════════════════

const char* WIFI_SSID     = "Noob";
const char* WIFI_PASSWORD = "holaamigo";

// Pin Definitions
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define I2C_SDA    21
#define I2C_SCL    22

// Motor Pins
#define FL_IN1 13
#define FL_IN2 12
#define FR_IN1 27
#define FR_IN2 26
#define BL_IN1 4
#define BL_IN2 25
#define BR_IN1 18
#define BR_IN2 19

// Navigation Parameters
#define WAYPOINT_TOLERANCE    2.5     // meters
#define HEADING_TOLERANCE     12.0    // degrees - must be within this to move forward
#define MIN_GPS_SPEED         0.3     // m/s
#define MIN_SATELLITES        6
#define MAX_WAYPOINTS         50

// Timing
#define GPS_UPDATE_INTERVAL        100
#define COMPASS_UPDATE_INTERVAL    100
#define WAYPOINT_PAUSE_MS          1000  // ★ 1 second pause at each waypoint

// ══════════════════════════ DATA STRUCTURES ══════════════════════════

struct Waypoint {
  double lat, lon;
  bool reached;
};

enum PathMode {
  MODE_NORMAL,   // Origin → waypoints → STOP
  MODE_RETURN,   // Origin → waypoints → Origin → STOP
  MODE_LOOP      // Origin → waypoints → Origin → REPEAT forever
};

// ══════════════════════════ GLOBAL OBJECTS ══════════════════════════

SFE_UBLOX_GNSS myGNSS;
IST8310 ist8310;
HardwareSerial gpsSerial(2);
WebServer server(80);

// ══════════════════════════ STATE VARIABLES ══════════════════════════

// GPS
double currentLat = 0, currentLon = 0;
double gpsHeading = 0, gpsSpeed = 0;
uint8_t satellites = 0;
bool gpsValid = false;

// Compass
double compassHeading = 0;
bool compassValid = false;
bool compassCalibrated = false;
unsigned long lastCompassUpdate = 0;

// Waypoints
Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;
int currentWP = 0;
double originLat = 0, originLon = 0;  // Store first waypoint as origin

// Navigation
bool navActive = false;
bool isWaiting = false;  // ★ Waiting at waypoint
unsigned long waitStartTime = 0;
PathMode pathMode = MODE_NORMAL;
bool returnPointAdded = false;

// State
String statusMsg = "Ready";
unsigned long lastGPSUpdate = 0;

// ══════════════════════════ MATH FUNCTIONS ══════════════════════════

double normalizeDegrees(double deg) {
  while (deg < 0) deg += 360;
  while (deg >= 360) deg -= 360;
  return deg;
}

double calcDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  double a = sin(dLat/2) * sin(dLat/2) + 
             cos(lat1 * PI/180) * cos(lat2 * PI/180) * 
             sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return 6371000.0 * c;
}

double calcBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = (lon2 - lon1) * PI / 180.0;
  double y = sin(dLon) * cos(lat2 * PI / 180.0);
  double x = cos(lat1 * PI/180) * sin(lat2 * PI/180) - 
             sin(lat1 * PI/180) * cos(lat2 * PI/180) * cos(dLon);
  return normalizeDegrees(atan2(y, x) * 180.0 / PI);
}

// ══════════════════════════ MOTOR CONTROL ══════════════════════════

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

void setMotors(int left, int right) {
  // Left motors
  if (left > 0) {
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
    digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  } else if (left < 0) {
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
    digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
  } else {
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
    digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, LOW);
  }
  
  // Right motors
  if (right > 0) {
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
  } else if (right < 0) {
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
  } else {
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
    digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, LOW);
  }
}

void stopMotors() {
  setMotors(0, 0);
}

// ══════════════════════════ COMPASS ══════════════════════════

void setupCompass() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(100);
  
  if (ist8310.setup(&Wire, NULL)) {
    compassValid = true;
    compassCalibrated = true;  // IST8310 already provides calibrated heading
    Serial.println("✓ Compass ready");
  } else {
    Serial.println("✗ Compass not found - will use GPS heading");
  }
}

void updateCompass() {
  if (!compassValid) return;
  
  unsigned long now = millis();
  if (now - lastCompassUpdate < COMPASS_UPDATE_INTERVAL) return;
  lastCompassUpdate = now;
  
  if (ist8310.update()) {
    compassHeading = normalizeDegrees(ist8310.get_heading_degrees());
  }
}

// ══════════════════════════ GPS ══════════════════════════

void setupGPS() {
  gpsSerial.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  if (!myGNSS.begin(gpsSerial)) {
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    if (myGNSS.begin(gpsSerial)) {
      myGNSS.setSerialRate(38400);
    }
  }
  myGNSS.setUART1Output(COM_TYPE_UBX);
  myGNSS.saveConfiguration();
  Serial.println("✓ GPS ready");
}

void updateGPS() {
  if (myGNSS.getPVT()) {
    satellites = myGNSS.getSIV();
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

// ══════════════════════════ NAVIGATION ══════════════════════════

// ★ Get current heading - COMPASS preferred, GPS fallback
double getCurrentHeading() {
  // PRIORITY 1: Compass (if valid and calibrated)
  if (compassValid && compassCalibrated) {
    return compassHeading;
  }
  
  // PRIORITY 2: GPS heading (if moving fast enough)
  if (gpsValid && gpsSpeed >= MIN_GPS_SPEED) {
    return gpsHeading;
  }
  
  // No valid heading
  return -1;
}

String getHeadingSource() {
  if (compassValid && compassCalibrated) return "COMPASS";
  if (gpsValid && gpsSpeed >= MIN_GPS_SPEED) return "GPS";
  return "NONE";
}

void navigate() {
  if (!navActive || !gpsValid) {
    stopMotors();
    return;
  }
  
  // ★ WAITING AT WAYPOINT (1 second pause)
  if (isWaiting) {
    if (millis() - waitStartTime >= WAYPOINT_PAUSE_MS) {
      isWaiting = false;
      currentWP++;
      
      // Check if path complete
      if (currentWP >= waypointCount) {
        handlePathComplete();
        return;
      }
      
      Serial.printf("\n→ Moving to WP %d/%d\n", currentWP + 1, waypointCount);
      statusMsg = "Aligning to WP" + String(currentWP + 1);
    }
    return;  // Still waiting
  }
  
  // Check if we've reached all waypoints
  if (currentWP >= waypointCount) {
    handlePathComplete();
    return;
  }
  
  Waypoint &target = waypoints[currentWP];
  
  // Calculate distance and bearing to target
  double distance = calcDistance(currentLat, currentLon, target.lat, target.lon);
  double bearingToTarget = calcBearing(currentLat, currentLon, target.lat, target.lon);
  
  // ★ Get current heading (COMPASS or GPS)
  double currentHeading = getCurrentHeading();
  String headingSource = getHeadingSource();
  
  // No valid heading? Nudge forward slowly
  if (currentHeading < 0) {
    setMotors(60, 60);  // Slow forward to establish GPS heading
    statusMsg = "Getting heading...";
    return;
  }
  
  // ★ CHECK IF WAYPOINT REACHED
  if (distance < WAYPOINT_TOLERANCE) {
    target.reached = true;
    stopMotors();
    
    Serial.printf("✓ Waypoint %d reached! (%.1fm)\n", currentWP + 1, distance);
    Serial.println("→ Waiting 1 second...");
    
    isWaiting = true;
    waitStartTime = millis();
    statusMsg = "✓ WP" + String(currentWP + 1) + " reached";
    return;
  }
  
  // ★ CALCULATE HEADING ERROR
  double headingError = bearingToTarget - currentHeading;
  
  // Normalize error to [-180, +180]
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  double absError = fabs(headingError);
  
  // ★ NAVIGATION LOGIC: ALIGN FIRST, THEN MOVE FORWARD
  
  if (absError > HEADING_TOLERANCE) {
    // NOT ALIGNED - TURN IN PLACE
    if (headingError > 0) {
      setMotors(150, -150);  // Turn right
      statusMsg = "Turn RIGHT " + String(absError, 1) + "° [" + headingSource + "]";
    } else {
      setMotors(-150, 150);  // Turn left
      statusMsg = "Turn LEFT " + String(absError, 1) + "° [" + headingSource + "]";
    }
  } else {
    // ALIGNED - MOVE FORWARD
    setMotors(255, 255);  // Full speed forward
    statusMsg = "→ WP" + String(currentWP + 1) + " " + String(distance, 1) + "m [" + headingSource + "]";
  }
}

void handlePathComplete() {
  switch (pathMode) {
    case MODE_NORMAL:
      // Just stop
      stopMotors();
      navActive = false;
      statusMsg = "✓ Path complete";
      Serial.println("\n✓ NORMAL mode: Path complete");
      break;
      
    case MODE_RETURN:
      // Add origin as return point (if not already added)
      if (!returnPointAdded) {
        waypoints[waypointCount].lat = originLat;
        waypoints[waypointCount].lon = originLon;
        waypoints[waypointCount].reached = false;
        waypointCount++;
        returnPointAdded = true;
        
        statusMsg = "Returning to origin";
        Serial.println("\n↩ RETURN mode: Returning to origin");
      } else {
        // Reached origin - stop
        stopMotors();
        navActive = false;
        waypointCount--;  // Remove return point
        returnPointAdded = false;
        statusMsg = "✓ Returned to origin";
        Serial.println("\n✓ RETURN mode: Back at origin");
      }
      break;
      
    case MODE_LOOP:
      // Reset and restart from beginning
      currentWP = 0;
      for (int i = 0; i < waypointCount; i++) {
        waypoints[i].reached = false;
      }
      statusMsg = "🔄 Looping";
      Serial.println("\n🔄 LOOP mode: Restarting from origin");
      break;
  }
}

// ══════════════════════════ WEB SERVER ══════════════════════════

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts++ < 20) {
    delay(500);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi: " + WiFi.localIP().toString());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("RoverAP", "rover12345");
    Serial.println("✓ AP: " + WiFi.softAPIP().toString());
  }
}

void handleRoot() {
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>GPS Rover v8.0</title>";
  h += "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>";
  h += "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>";
  h += "<style>";
  h += "body{margin:0;font-family:Arial;background:#1a1a1a;color:#fff}";
  h += ".container{display:grid;grid-template-columns:1fr 360px;height:100vh}";
  h += "#map{height:100%}";
  h += ".panel{padding:15px;overflow-y:auto;background:#2a2a2a;border-left:2px solid #444}";
  h += "h2{margin:0 0 15px;color:#4a9eff}";
  h += ".section{margin:15px 0;padding:12px;background:#333;border-radius:8px}";
  h += ".stat{display:flex;justify-content:space-between;margin:8px 0;font-size:14px}";
  h += ".stat span:first-child{color:#aaa}";
  h += ".btn{width:100%;padding:12px;margin:5px 0;border:none;border-radius:6px;font-size:14px;font-weight:600;cursor:pointer;transition:.2s}";
  h += ".btn-green{background:#28a745;color:#fff}.btn-green:hover{background:#218838}";
  h += ".btn-red{background:#dc3545;color:#fff}.btn-red:hover{background:#c82333}";
  h += ".btn-blue{background:#007bff;color:#fff}.btn-blue:hover{background:#0056b3}";
  h += ".mode-btns{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:10px 0}";
  h += ".mode-btn{padding:10px;background:#555;border:none;border-radius:6px;color:#fff;font-size:13px;cursor:pointer;transition:.2s}";
  h += ".mode-btn:hover{background:#666}";
  h += ".mode-btn.active{background:#007bff}";
  h += ".arrows{display:grid;grid-template-columns:repeat(3,1fr);gap:5px;margin:10px 0}";
  h += ".arrow{padding:18px;background:#555;border:none;border-radius:6px;color:#fff;font-size:20px;cursor:pointer;user-select:none}";
  h += ".arrow:active{background:#007bff;transform:scale(.95)}";
  h += "@media(max-width:768px){.container{grid-template-columns:1fr;grid-template-rows:60vh 1fr}}";
  h += "</style></head><body>";
  
  h += "<div class='container'>";
  h += "<div id='map'></div>";
  h += "<div class='panel'>";
  
  h += "<h2>🚗 GPS Rover v8.0</h2>";
  
  h += "<div class='section'>";
  h += "<div class='stat'><span>GPS:</span><span id='gps'>—</span></div>";
  h += "<div class='stat'><span>Heading:</span><span id='hdg'>—</span></div>";
  h += "<div class='stat'><span>Waypoints:</span><span id='wpc'>0</span></div>";
  h += "<div class='stat'><span>Status:</span><span id='msg'>Ready</span></div>";
  h += "</div>";
  
  h += "<div class='section'>";
  h += "<div style='font-size:12px;color:#aaa;margin-bottom:8px'>Path Mode:</div>";
  h += "<div class='mode-btns'>";
  h += "<button class='mode-btn active' id='modeNormal' onclick='setMode(0)'>Normal</button>";
  h += "<button class='mode-btn' id='modeReturn' onclick='setMode(1)'>Return</button>";
  h += "<button class='mode-btn' id='modeLoop' onclick='setMode(2)'>Loop</button>";
  h += "</div></div>";
  
  h += "<div class='section'>";
  h += "<button class='btn btn-green' onclick='start()'>▶ START</button>";
  h += "<button class='btn btn-red' onclick='stop()'>⏹ STOP</button>";
  h += "<button class='btn btn-blue' onclick='clear()'>Clear Waypoints</button>";
  h += "</div>";
  
  h += "<div class='section'>";
  h += "<div style='font-size:12px;color:#aaa;margin-bottom:8px'>Manual Control:</div>";
  h += "<div class='arrows'>";
  h += "<div></div>";
  h += "<button class='arrow' onmousedown='move(\"f\")' onmouseup='move(\"s\")' ontouchstart='move(\"f\")' ontouchend='move(\"s\")'>▲</button>";
  h += "<div></div>";
  h += "<button class='arrow' onmousedown='move(\"l\")' onmouseup='move(\"s\")' ontouchstart='move(\"l\")' ontouchend='move(\"s\")'>◀</button>";
  h += "<button class='arrow' onclick='move(\"s\")' style='background:#dc3545'>■</button>";
  h += "<button class='arrow' onmousedown='move(\"r\")' onmouseup='move(\"s\")' ontouchstart='move(\"r\")' ontouchend='move(\"s\")'>▶</button>";
  h += "<div></div>";
  h += "<button class='arrow' onmousedown='move(\"b\")' onmouseup='move(\"s\")' ontouchstart='move(\"b\")' ontouchend='move(\"s\")'>▼</button>";
  h += "</div></div>";
  
  h += "</div></div>";
  
  h += "<script>";
  h += "var map,rover,wps=[];";
  h += "map=L.map('map').setView([0,0],2);";
  h += "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);";
  h += "var rIcon=L.divIcon({html:'<div style=\"width:24px;height:24px;background:#ef4444;border:3px solid #fff;border-radius:50%;box-shadow:0 2px 8px rgba(0,0,0,.4)\"></div>',iconSize:[24,24],iconAnchor:[12,12]});";
  h += "rover=L.marker([0,0],{icon:rIcon}).addTo(map);";
  h += "map.on('click',e=>fetch('/add?lat='+e.latlng.lat+'&lon='+e.latlng.lng));";
  
  h += "function update(){";
  h += "fetch('/status').then(r=>r.json()).then(d=>{";
  h += "document.getElementById('gps').innerText=d.gps?'LOCK ('+d.sat+')':'NO FIX';";
  h += "document.getElementById('hdg').innerText=d.hdg+'° ['+d.src+']';";
  h += "document.getElementById('wpc').innerText=d.wpc;";
  h += "document.getElementById('msg').innerText=d.msg;";
  h += "if(d.gps){rover.setLatLng([d.lat,d.lon]);if(wps.length==0)map.setView([d.lat,d.lon],17);}";
  h += "wps.forEach(m=>map.removeLayer(m));wps=[];";
  h += "d.wps.forEach((w,i)=>{";
  h += "var col=w.r?'#6b7280':'#3b82f6';";
  h += "var ic=L.divIcon({html:'<div style=\"width:20px;height:20px;background:'+col+';border:2px solid #fff;border-radius:50%;display:flex;align-items:center;justify-content:center;color:#fff;font-size:11px;font-weight:700\">'+(i+1)+'</div>',iconSize:[20,20],iconAnchor:[10,10]});";
  h += "var m=L.marker([w.lat,w.lon],{icon:ic}).addTo(map).bindPopup('WP'+(i+1)+(w.r?' ✓':''));";
  h += "wps.push(m);";
  h += "});";
  h += "});}";
  
  h += "function start(){fetch('/start').then(r=>r.text()).then(t=>{if(t.includes('Error'))alert(t);});}";
  h += "function stop(){fetch('/stop');}";
  h += "function clear(){if(confirm('Clear all waypoints?'))fetch('/clear').then(()=>{wps.forEach(m=>map.removeLayer(m));wps=[];});}";
  h += "function move(d){fetch('/move?d='+d);}";
  h += "function setMode(m){";
  h += "fetch('/mode?m='+m).then(()=>{";
  h += "document.querySelectorAll('.mode-btn').forEach(b=>b.classList.remove('active'));";
  h += "var ids=['modeNormal','modeReturn','modeLoop'];";
  h += "document.getElementById(ids[m]).classList.add('active');";
  h += "});";
  h += "}";
  
  h += "setInterval(update,500);";
  h += "update();";
  h += "</script></body></html>";
  
  server.send(200, "text/html", h);
}

void handleStatus() {
  String json = "{";
  json += "\"gps\":" + String(gpsValid ? "true" : "false") + ",";
  json += "\"sat\":" + String(satellites) + ",";
  json += "\"lat\":" + String(currentLat, 7) + ",";
  json += "\"lon\":" + String(currentLon, 7) + ",";
  json += "\"hdg\":" + String((int)getCurrentHeading()) + ",";
  json += "\"src\":\"" + getHeadingSource() + "\",";
  json += "\"wpc\":" + String(waypointCount) + ",";
  json += "\"msg\":\"" + statusMsg + "\",";
  json += "\"wps\":[";
  for (int i = 0; i < waypointCount; i++) {
    if (i > 0) json += ",";
    json += "{\"lat\":" + String(waypoints[i].lat, 6) + ",";
    json += "\"lon\":" + String(waypoints[i].lon, 6) + ",";
    json += "\"r\":" + String(waypoints[i].reached ? "true" : "false") + "}";
  }
  json += "]}";
  
  server.send(200, "application/json", json);
}

void handleAdd() {
  if (waypointCount >= MAX_WAYPOINTS) {
    server.send(400, "text/plain", "Error: Waypoint limit reached");
    return;
  }
  
  waypoints[waypointCount].lat = server.arg("lat").toDouble();
  waypoints[waypointCount].lon = server.arg("lon").toDouble();
  waypoints[waypointCount].reached = false;
  
  // Store first waypoint as origin
  if (waypointCount == 0) {
    originLat = waypoints[0].lat;
    originLon = waypoints[0].lon;
  }
  
  waypointCount++;
  Serial.printf("WP %d added: %.6f, %.6f\n", waypointCount, 
                waypoints[waypointCount-1].lat, waypoints[waypointCount-1].lon);
  
  server.send(200, "text/plain", "OK");
}

void handleClear() {
  waypointCount = 0;
  currentWP = 0;
  navActive = false;
  isWaiting = false;
  returnPointAdded = false;
  stopMotors();
  Serial.println("Waypoints cleared");
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  if (waypointCount == 0) {
    server.send(400, "text/plain", "Error: No waypoints");
    return;
  }
  
  if (!gpsValid) {
    server.send(400, "text/plain", "Error: GPS not ready");
    return;
  }
  
  currentWP = 0;
  navActive = true;
  isWaiting = false;
  returnPointAdded = false;
  
  for (int i = 0; i < waypointCount; i++) {
    waypoints[i].reached = false;
  }
  
  Serial.println("\n▶ Navigation started");
  Serial.printf("Mode: %s\n", pathMode == MODE_NORMAL ? "NORMAL" : 
                               pathMode == MODE_RETURN ? "RETURN" : "LOOP");
  Serial.printf("Waypoints: %d\n", waypointCount);
  
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  navActive = false;
  isWaiting = false;
  stopMotors();
  Serial.println("⏹ Navigation stopped");
  server.send(200, "text/plain", "OK");
}

void handleMove() {
  String d = server.arg("d");
  navActive = false;
  
  if (d == "f") setMotors(255, 255);
  else if (d == "b") setMotors(-255, -255);
  else if (d == "l") setMotors(-150, 150);
  else if (d == "r") setMotors(150, -150);
  else if (d == "s") stopMotors();
  
  server.send(200, "text/plain", "OK");
}

void handleMode() {
  int m = server.arg("m").toInt();
  
  if (m == 0) pathMode = MODE_NORMAL;
  else if (m == 1) pathMode = MODE_RETURN;
  else if (m == 2) pathMode = MODE_LOOP;
  
  Serial.printf("Mode changed to: %s\n", pathMode == MODE_NORMAL ? "NORMAL" : 
                                          pathMode == MODE_RETURN ? "RETURN" : "LOOP");
  
  server.send(200, "text/plain", "OK");
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/add", handleAdd);
  server.on("/clear", handleClear);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/move", handleMove);
  server.on("/mode", handleMode);
  server.begin();
  Serial.println("✓ Web server ready");
}

// ══════════════════════════ SETUP ══════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔═════════════════════════════════════════╗");
  Serial.println("║  ESP32 GPS Rover v8.0 - CLEAN VERSION  ║");
  Serial.println("╚═════════════════════════════════════════╝\n");
  
  setupMotors();
  setupCompass();
  setupGPS();
  setupWiFi();
  setupWebServer();
  
  Serial.println("\n✓ System ready");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("MODES:");
  Serial.println("  NORMAL: Origin → WP1 → WP2 → STOP");
  Serial.println("  RETURN: Origin → WP1 → WP2 → Origin → STOP");
  Serial.println("  LOOP:   Origin → WP1 → WP2 → Origin → REPEAT");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("Web: http://" + WiFi.localIP().toString());
  Serial.println();
}

// ══════════════════════════ MAIN LOOP ══════════════════════════

void loop() {
  server.handleClient();
  
  // Update sensors
  unsigned long now = millis();
  if (now - lastGPSUpdate >= GPS_UPDATE_INTERVAL) {
    updateGPS();
    updateCompass();
    lastGPSUpdate = now;
  }
  
  // Navigate
  if (navActive) {
    navigate();
  }
}
