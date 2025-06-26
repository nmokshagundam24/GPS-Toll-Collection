#include <LiquidCrystal.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Constants
static const int RXPin = 10;
static const int TXPin = -1;
static const uint32_t GPSBaud = 9600;
static const double EarthRadiusKm = 6371.0;
static const double ThresholdMeters = 10.0; // Threshold to ignore small movements

// Objects
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Variables
double firstLat = 0.0;
double firstLon = 0.0;
double previousLat = 0.0;
double previousLon = 0.0;
double totalDistance = 0.0;
double distanceFromStart = 0.0;
bool firstData = true;
unsigned long lastTime = 0;
int checkpoint_reached = 0;
double checkpoint_lat = 17.4118;
double checkpoint_lon = 78.3987;

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  lcd.begin(16, 2);
  lcd.print("Initializing...");
}

void loop() {
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      displayInfo();
    }
  }

  if (millis() - lastTime > 10000) {
    if (gps.charsProcessed() < 10) {
      Serial.println(F("No GPS detected: check wiring."));
      lcd.clear();
      lcd.print("GPS not working");
      while (true);  // Halting the system as GPS is not working
    }
  }
}

void displayInfo() {
  if (gps.location.isValid()) {
    double currentLat = round(gps.location.lat() * 10000.0) / 10000.0;
    double currentLon = round(gps.location.lng() * 10000.0) / 10000.0;

    if (firstData) {
      firstLat = currentLat;
      firstLon = currentLon;
      previousLat = currentLat;
      previousLon = currentLon;
      firstData = false;
    } else if (checkpoint_reached == 0) {
      double distance = calculateDistance(previousLat, previousLon, currentLat, currentLon);
      if (distance >= ThresholdMeters) {
        totalDistance += distance;
        previousLat = currentLat;
        previousLon = currentLon;
      }
      distanceFromStart = calculateDistance(firstLat, firstLon, currentLat, currentLon);

      double distance_checkpoint = calculateDistance(checkpoint_lat, checkpoint_lon, currentLat, currentLon);
      if (distance_checkpoint <= ThresholdMeters) {
        checkpoint_reached = 1;
      }
    }

    // Toll fare calculation
    double toll_fare_per_m = 0.25;
    double total_fare = toll_fare_per_m * totalDistance;
    Serial.print("Amount: ");
    Serial.print(total_fare);
    Serial.println(" Rs");

    Serial.print("Distance from Start: ");
    Serial.print(distanceFromStart);
    Serial.print(" m  Lat: ");
    Serial.print(currentLat, 4);
    Serial.print("  Lon: ");
    Serial.println(currentLon, 4);

    if (gps.date.isValid()) {
      Serial.print("  Date: ");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.println(gps.date.year());
    } else {
      Serial.println("  Date: INVALID");
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(totalDistance, 2);  // Display distance with 2 decimal places
    lcd.print("m");

    lcd.setCursor(0, 1);
    lcd.print("Amount: ");
    lcd.print(total_fare, 2);  // Display amount with 2 decimal places
    lcd.print(" Rs");

  } else {
    Serial.println(F("GPS location invalid."));
    lcd.clear();
    lcd.print("GPS invalid");
  }
}

double toRadians(double degrees) {
  return degrees * PI / 180.0;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  lat1 = toRadians(lat1);
  lon1 = toRadians(lon1);
  lat2 = toRadians(lat2);
  lon2 = toRadians(lon2);

  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;
  double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  double c = 2 * asin(sqrt(a));

  return EarthRadiusKm * c * 1000;  // Convert to meters
}
