#include "DHT.h" 
#include <Wire.h>
#include <SoftwareSerial.h>
#define RX_PIN 10 
#define TX_PIN 11 


SoftwareSerial espSerial(RX_PIN, TX_PIN);




// Pin and threshold definitions
const int Vibration_signal = A2;  // Analog pin for the vibration sensor
const int vibrationThreshold = 500; // Threshold for analog value to detect vibration
const int vibrationEventThreshold = 5; // Number of vibration events to trigger detection
const unsigned long debounceTime = 200; // Minimum time (ms) between valid detections

// Variables for tracking  vibration
int vibrationCount = 0;
unsigned long lastVibrationTime = 0;

//.....................rPM
const int sensorPin = A3;           // Analog pin connected to the IR sensor
const int threshold = 512;         // Threshold for detecting a signal (adjust as needed)
unsigned long lastTime = 0;        // Last time a pulse was detected
unsigned long currentTime = 0;     // Current time for calculation
volatile int pulseCount = 0;       // Number of pulses detected
unsigned long rpm = 0;             // Calculated RPM
unsigned long timeElapsed = 0;     // Time elapsed since the last pulse
unsigned long lastCalculationTime = 0; // Time of last RPM calculation

//.................RPM END

 
 //............................motor
#define ENA 4  // PWM pin for motor speed control
#define IN1 2  // Motor direction control pin 1
#define IN2 3  // Motor direction control pin 2

//....................motor end
 
 //...............................buzzer
#define IN3 8  // Pin connected to IN3 on the motor driver
#define IN4 7  // Pin connected to IN4 on the motor driver
#define ENB 6  
//..............................buzzer end


//........ ............dht
#define DHTPIN A0      // Pin connected to DHT11 data pin (A5 used as digital pin)
#define DHTTYPE DHT11   // DHT 11 sensor type

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);
//.....................DHTend





 String data;


String data_for_firebase="";
//overspeeding

  #define BUTTON_PIN_alcohol 10

  #define BUTTON_PIN 9
  #define SPEED_CHECK_INTERVAL 5000 // 5 seconds
  // Track button press timing
  unsigned long buttonPressStart = 0;
  bool overspeedWarning = false;
//overspeeding end

void setup() {


  Serial.begin(115200);


 

  // overspeed button
  pinMode(BUTTON_PIN, INPUT);
  // oversppedd 
  pinMode(BUTTON_PIN_alcohol, INPUT);
  

 

	

//.........................rpM
 pinMode(sensorPin, INPUT);     
//....................RPM END


 //................motor
  pinMode(ENA, OUTPUT);  // Set ENA as an output for PWM control
  pinMode(IN1, OUTPUT);  // Set IN1 as output for motor direction
  pinMode(IN2, OUTPUT);  // Set IN2 as output for motor direction

 //..............motor e

   // Initialize buzzer 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  ///...................buzeer end

  //........ ............dht
  dht.begin();           // Initialize DHT sensor
  Serial.println("DHT11 Sensor Ready!");
//.....................DHTend


//.....................vibration
 pinMode(Vibration_signal, INPUT); // Set pin as input
//....................vibration end

}

void loop() {

motorr();

dhT();
vibration();
RPM();
check();
eyeCheck();
alcohol_detect();





delay(200);
 Serial.println(data_for_firebase);
 data_for_firebase="";

buzzer_off();
 
}






void buzzer(){

 // Turn the buzzer ON
  Serial.println("Buzzer ON");
  digitalWrite(IN3, HIGH);  
  digitalWrite(IN4, LOW);   
  analogWrite(ENB, 255);   
 



}
void motorr(){

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

 

    analogWrite(ENA, 255);  
  
  Serial.println("car is running");




}

void gps(){


   
   float latitude = 33.622368;
   float longitude = 72.956559;

    Serial.print("Latitude: ");
    Serial.print(latitude, 6);  
    Serial.print(" Longitude: ");
    Serial.println(longitude, 6);  



}


void alcohol_detect(){



    static unsigned long buttonPressStartTime = 0; // To track button press time
   
    bool state=false;
    while (digitalRead(BUTTON_PIN_alcohol) == LOW) { // Until the button is released
      
        if (buttonPressStartTime == 0) {
           
            buttonPressStartTime = millis();
        }

        Serial.println("WARNING: alcohol detected");
        buzzer();

        if ((millis() - buttonPressStartTime) >= 5000) { // If held for 5 seconds
            Serial.println("THRESHOLD REACHED... millis() - buttonPressStartTime = ");
            Serial.println(millis() - buttonPressStartTime);
            state=true;
            
            for (int i = 255; i >= 0; i -= 5) {
                analogWrite(ENA, i);  // Set motor speed
                  Serial.println("stoping car");
                delay(50);           // Smooth speed reduction
            }

            buzzer_off();
            buttonPressStartTime = 0; // Reset the timer
            delay(5000);

           
            break; // Exit the while loop
        }
    }

    // Reset `buttonPressStartTime` if the button is released
    if (digitalRead(BUTTON_PIN_alcohol) == HIGH) {
        buttonPressStartTime = 0;
    }

    if(state){
      data_for_firebase += ";DRUNK_STATE: Person is Drunk "; 

    }else{
      data_for_firebase += ";DRUNK_STATE: NO "; 

      
    }
}

void eyeCheck(){

    static unsigned long eyeclosedTime = 0; // To track eye closed   time
   
    bool sLeep=false;
       
   if(Serial.available() && (Serial.readStringUntil('\n')).toFloat() < 0.5 ){
     data = Serial.readStringUntil('\n');
    
     Serial.println(data);

   }else{
    data_for_firebase += ";sleepState: no ";
    return;

   }

    while ( data.toFloat() < 0.5  ) {
      
        if (eyeclosedTime == 0) {
            
            eyeclosedTime = millis();
        }

        Serial.println("WARNING: SLEEPING DETECTED");
        buzzer();

        if ((millis() - eyeclosedTime) >= 3000) { 
            Serial.println("THRESHOLD REACHED... millis() - eyeclosedTime = ");
            Serial.println(millis() - eyeclosedTime);
            sLeep=true;
          
            // Gradually reduce speed
            for (int i = 255; i > 100; i -= 5) {
                analogWrite(ENA, i);  // Set motor speed
                Serial.println(" Reducing the car speed ");
                delay(50);           // Smooth speed reduction
            }

            buzzer_off();
            eyeclosedTime = 0; // Reset the timer
            delay(2000);
            break; 
        }

        if(Serial.available()){
             data = Serial.readStringUntil('\n');
             Serial.println(data);

         }else{

          return;

         }

    }

    if(sLeep){
   data_for_firebase += ";sleepState: Person is sleeping ";

    }else{

       
    }

}

void buzzer_off(){

 // Turn the buzzer ON
  Serial.println("Buzzer off");
  digitalWrite(IN3, HIGH);  
  digitalWrite(IN4, LOW);   
  analogWrite(ENB, 0);    // Maximum power to the buzzer
 



}



void dhT(){
 // Read temperature as Celsius
  float temperature = dht.readTemperature();
  // Read humidity
  float humidity = dht.readHumidity();

  
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    data_for_firebase += "TEMP: Failed to read from DHT sensor!";
    return;
  }

  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  data_for_firebase += "TEMP:"+String(temperature, 2)+" °C";
  
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");



}

void check() {
    

    static unsigned long buttonPressStartTime = 0; 

    bool overspeed= false;
    

    while (digitalRead(BUTTON_PIN) == LOW) { 
      
        if (buttonPressStartTime == 0) {
      
            buttonPressStartTime = millis();
        }

        Serial.println("WARNING: OVERSPEEDING");
        buzzer();

        if ((millis() - buttonPressStartTime) >= 5000) {
            Serial.println("THRESHOLD REACHED... millis() - buttonPressStartTime = ");
            Serial.println(millis() - buttonPressStartTime);
            overspeed=true;
          
            for (int i = 255; i > 100; i -= 5) {
                analogWrite(ENA, i);  
                delay(500);           // Smooth speed reduction
            }

            buzzer_off();
            buttonPressStartTime = 0; // Reset the timer
            delay(5000);

            
            break; 
        }
    }


    if (digitalRead(BUTTON_PIN) == HIGH) {
        buttonPressStartTime = 0;
    }

    if(overspeed){
   
     data_for_firebase += ";overspeed: THE DRIVER OVERSPEED AND CAR WAS STOPED";
    }else{
       data_for_firebase += ";overspeed: no";

    }
}



void vibration() {
 
  int analogValue = analogRead(Vibration_signal);

   Serial.println(analogValue);

  

   if(analogValue>600){

    Serial.println("Collision  Detected");
     data_for_firebase += ";VIB: vibration detected be carefull AT";
   }else{

        data_for_firebase += ";VIB:"+String(analogValue);
   }
  
}

void RPM(){

 int sensorValue = analogRead(sensorPin);  

  data_for_firebase += ";RPM:"+String(sensorValue);

  bool currentState = (sensorValue > threshold);

  // Detect rising edge
  if (currentState && (millis() - lastTime > 50)) { 
    pulseCount++;                    // Increment pulse count
    lastTime = millis();             
  }

 
  currentTime = millis();
  if (currentTime - lastCalculationTime >= 1000) {
    rpm = (pulseCount * 60) / 2;     
    pulseCount = 0;                  
    lastCalculationTime = currentTime; 
   
  }

}

