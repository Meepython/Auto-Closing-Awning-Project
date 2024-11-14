#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
#define LED_RED 12 // The pin the Red LED is connected to 12 pin
#define LED_GREEN 13 // The pin the Green LED is connected to 13 pin

bool ButtonPress = false;
bool IsOpen = false;
bool IsWind = false;

const int buttonPin = 2;   // The number of the pushbutton pin
int buttonState = 0;    // Variable for reading the pushbutton status

const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum

int sensorReading;
int range;
const int Rain_D = 4; // The number of the rain sensor digital pin

int light;

const int RecordTime = 3; //Define Measuring Time (Seconds)
const int SensorPin = 3;  //Define Interrupt Pin (2 or 3 @ Arduino Uno)

int InterruptCounter;
float WindSpeed;

const int buzzer = 8;

const int relay1 = 6;  // the Arduino pin, which connects to the IN pin of relay
const int relay2 = 7;  // the Arduino pin, which connects to the IN pin of relay

const int FrontSensor = 11;
const int BackSensor = 10;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);  // Set the button pin as an input
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT);  // Set the led as an output
  pinMode(A0, INPUT); // Light(LDR) analog pin A0
  pinMode(Rain_D, INPUT); // Rain Sensor
  pinMode(buzzer, OUTPUT); // Buzzer
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay1, HIGH);
  pinMode(relay2, HIGH);
  pinMode(FrontSensor, INPUT);// Front IR sensor
  pinMode(BackSensor, INPUT); // Back IR sensor
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW); // LED
  lcd.init(); lcd.backlight();
  delay(1000);
  lcd.setCursor(1, 0); lcd.print("AIA System");
  delay(1000);
  lcd.setCursor(1, 1); lcd.print("  Welcome  ");
  delay(1000);
  lcd.clear();
  lcd.setCursor(1, 0); lcd.print(" Version 1.1");
  lcd.setCursor(1, 1); lcd.print("Date 1/9/2022");
  delay(1000);
  lcd.clear();
}

void loop() {
  lcd.println("Reset");
  light = analogRead(A0);
  sensorReading = analogRead(A1);
  range = map(sensorReading, sensorMin, sensorMax, 0, 3);
  buttonState = digitalRead(buttonPin); // Read the state of the pushbutton value
  meassure();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("WS:"); lcd.print(WindSpeed); lcd.print("km/h");
  lcd.setCursor(0, 1); lcd.print("LDR:"); lcd.print(light); lcd.write(3); lcd.print(sensorReading);

  if (buttonState == HIGH) { // If button is pressed
    ButtonFunction();
  }
  while ( WindSpeed >= 7.00 && IsOpen == true && IsWind == false) {
    //Buzzer();
    lcd.clear(); lcd.setCursor(1, 0); lcd.print("Wind Warning!");
    CloseAwning();
    delay(1000);
    IsOpen = !IsOpen;
    IsWind = !IsWind;
  }
  while ( WindSpeed < 7.00 && IsWind == true) {
    lcd.clear(); lcd.setCursor(1, 0); lcd.print("Wind is gone");
    IsWind = !IsWind;
    delay(1000);
  }
  if (buttonState % 2 == 0 && !ButtonPress && !IsWind ) {
    while ((light > 960 || range == 0 || range == 1) && IsWind == false && !IsOpen) {
      delay(1000);
      OpenAwning();
      IsOpen = !IsOpen;
    }
    while ( light < 960 && range == 2 && IsOpen) {
      delay(1000);
      CloseAwning();
      IsOpen = !IsOpen;
    }
    switch (range) {
      case 0: // Sensor getting wet
        Serial.println("Flood");
        lcd.clear();
        lcd.setCursor(1, 0); lcd.print("Flood warn");
        break;
      case 1: // Sensor also getting wet
        Serial.println("Rain Warning");
        lcd.clear();
        lcd.setCursor(1, 0); lcd.print("Rain warn");
        break;
      case 2: // Sensor dry cuz no rain
        Serial.println("Not Raining");
        break;
    }
  }
  delay(200);
}
void ButtonFunction() {
  lcd.clear(); lcd.setCursor(1, 0); lcd.print("Button press");
  Serial.println("Button press");
  if (!IsOpen) {
    if (IsWind) {
      lcd.clear(); lcd.setCursor(1, 0); lcd.print("Wind caution"); lcd.setCursor(1, 1); lcd.print("Too dangerous to open");
    }
    else {
      digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, LOW);
      //Buzzer();
      delay(1000);
      OpenAwning();
      IsOpen = !IsOpen;
      ButtonPress = true;
    }
  }
  else if (IsOpen) {
    digitalWrite(LED_GREEN, LOW) ; digitalWrite(LED_RED, HIGH);
    //Buzzer();
    delay(1000);
    CloseAwning();
    IsOpen = !IsOpen;
    ButtonPress = false;
  }
}
void OpenAwning() { //Open awning function
  digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, LOW);
  while (digitalRead(2) == HIGH) {
    MotorForward();
  }
  MotorStop();
}
//Close awning function
void CloseAwning() {
  digitalWrite(LED_GREEN, LOW); digitalWrite(LED_RED, HIGH);
  while (digitalRead(3) == HIGH ) { //Motor'll backward until the IR sensor will find as object
    MotorBackward();
  }
  MotorStop();
}
void meassure() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING);
  delay(1000 * RecordTime);
  detachInterrupt(digitalPinToInterrupt(SensorPin));
  WindSpeed = (float)InterruptCounter / (float)RecordTime * 2.4;
}
void countup() {
  InterruptCounter++;
}
void Buzzer() {
  tone(buzzer, 1000); // Send 1KHz sound signal...
  delay(500);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(500);        // ...for 1sec
}
void MotorForward() { //Move the motor forward
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, LOW); //After that the motor'll stop
}
void MotorBackward() { //Move the motor backward
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, HIGH);
}
void MotorStop() { //Stop the motor
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
}
