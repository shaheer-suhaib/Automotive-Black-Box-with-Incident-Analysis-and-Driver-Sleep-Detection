#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Wire.h> // Include the Wire library for I2C
#include <LiquidCrystal_I2C.h> // Include the LCD library for I2C

#include <TinyGPS++.h>

// The TinyGPS++ object
TinyGPSPlus gps;


// GPS module pins
#define GPS_RX 16  // GPS TX to ESP32 RX2
#define GPS_TX 17  // GPS RX to ESP32 TX2

// Arduino communication pins
#define ARDUINO_RX 3  // Arduino TX to ESP32 RX1
#define ARDUINO_TX 1  // Arduino RX to ESP32 TX1

// Create separate Serial instances
HardwareSerial gpsSerial(2);      // UART2 for GPS
HardwareSerial arduinoSerial(1); // UART1 for Arduin



// Wi-Fi and Firebase credentials
#define FIREBASE_HOST "mmd-project-8d860-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "AIzaSyA02V91yHJtldj48IcLun6QC6znGLIn-IQ"
#define WIFI_SSID "higj"
#define WIFI_PASSWORD "12345678"


// Firebase and Serial setup
FirebaseData firebaseData;
FirebaseJson Json;
FirebaseConfig config;
FirebaseAuth auth;

// LCD initialization
LiquidCrystal_I2C lcd(0x27, 16, 2); // Replace "0x27" with your LCD's I2C address

// Quotes array
String quotes[][2] = {
  {"It's better to", "rest if tired"},
  {"Keep your eyes", "on the road"},
  {"Drive safe,", "stay safe!"},
  {"Don't rush,", "enjoy the ride"},
  {"Take a break,", "relax a bit"}
};

// Timing variables for lcd_Display()
unsigned long previousMillisLCD = 0;
const long intervalLCD = 2000; // 2 seconds

String sleepState = "no", drunkState = "no", overspeed = "no";
int vibration = 100; // Default vibration value
String Latitude, Longitude;
void setup() {
  Serial.begin(115200);
    gpsSerial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);

  // Initialize UART1 for Arduino
  arduinoSerial.begin(115200, SERIAL_8N1, ARDUINO_RX, ARDUINO_TX);

  Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    lcd.setCursor(0, 1);
    lcd.print("WiFi...");
  }
  Serial.println("Connected to WiFi.");
  lcd.setCursor(0, 1);
  lcd.print("WiFi Connected");

  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
 

  Serial.println("GPS Module Test");

  Serial.println("Setup complete.");
  lcd.setCursor(0, 0);
  lcd.print("Setup Complete");
  lcd.setCursor(0, 1);
  lcd.print("Ready...");
}

void parseAndSendData(String data) {
  // Check if the data starts with "TEMP:"
  if (data.startsWith("TEMP:")) {
    // Extract TEMP (after "TEMP:")
    int tempStart = 5;
    int tempEnd = data.indexOf(";", tempStart);
    String temp = data.substring(tempStart, tempEnd);
    
    // Extract VIB (after "VIB:")
    int vibStart = data.indexOf("VIB:") + 4;
    int vibEnd = data.indexOf(";", vibStart);
    String vib = data.substring(vibStart, vibEnd);
    
   // Extract RPM (after "RPM:")
    int rpmStart = data.indexOf("RPM:") + 4;
    int rpmEnd = data.indexOf(";", rpmStart);
    String rpm = data.substring(rpmStart, rpmEnd);

    // Extract overspeed (after "overspeed:")
    int overspeedStart = data.indexOf("overspeed:") + 10;
    int overspeedEnd = data.indexOf(";", overspeedStart);
    String overspeed = data.substring(overspeedStart, overspeedEnd);

    // Extract sleepState (after "sleepState:")
    int sleepStateStart = data.indexOf("sleepState:") + 11;
    int sleepStateEnd = data.indexOf(";", sleepStateStart);
    String sleepState = data.substring(sleepStateStart, sleepStateEnd);

    // Extract DRUNK_STATE (after "DRUNK_STATE:")
    int drunkStateStart = data.indexOf("DRUNK_STATE:") + 12;
    String drunkState = data.substring(drunkStateStart);

    // Print the valid data to Serial Monitor
    Serial.println("Valid Data:");
    Serial.println("Temperature: " + temp);
    Serial.println("Vibration: " + vib);
    Serial.println("RPM: " + rpm);
    Serial.println("Overspeed: " + overspeed);
    Serial.println("Sleep State: " + sleepState);
    Serial.println("Drunk State: " + drunkState);

    // Send valid data to Firebase
    Json.clear(); // Clear previous data in JSON
    Json.set("Temperature", temp);  // Set new data
    Json.set("Vibration", vib);     // Set new data
    Json.set("RPM", rpm);           // Set new data
    Json.set("Overspeed", overspeed); // Set new data
    Json.set("SleepState", sleepState); // Set new data
    Json.set("DrunkState", drunkState); // Set new data
    Json.set("GPS  >> Lattitude ", Latitude); // Set new data
    Json.set("GPS  >> Longitude", Longitude); // Set new data

    if (Firebase.updateNode(firebaseData, "/SensorData", Json)) {
      Serial.println("Data successfully sent to Firebase.");
    } else {
      Serial.print("Error sending data to Firebase: ");
      Serial.println(firebaseData.errorReason());
    }
  } else {
    Serial.println("Invalid Data (skipped): " + data);
  }
}


void lcd_Display() {
  lcd.clear();
  int quoteIndex = random(0, 5); // Randomly pick a quote index
  lcd.setCursor(0, 0);
  lcd.print(quotes[quoteIndex][0]);
  lcd.setCursor(0, 1);
  lcd.print(quotes[quoteIndex][1]);
}


void loop() {
  // GPS data handling
  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Latitude = String(gps.location.lat(), 6);
        Longitude = String(gps.location.lng(), 6);
        Serial.println("Latitude: " + Latitude);
        Serial.println("Longitude: " + Longitude);
      } else {
        Serial.println("Waiting for GPS signal...");
      }
    }
  }

  // Arduino data handling
  if (arduinoSerial.available() > 0) {
    String receivedData = arduinoSerial.readStringUntil('\n');
    Serial.println("Received from Arduino: " + receivedData);
    parseAndSendData(receivedData);
  }


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisLCD >= intervalLCD) {
    previousMillisLCD = currentMillis;
    lcd_Display();
  }
}