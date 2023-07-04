/*
 * Date Created:    June 25th, 2023
 * Last Modified:   July 4th, 2023
 * Filename:        heart_rate_detection.ino
 * Purpose:         Detect heart rate using the MAX30105 sensor and display it on an OLED display.
 * Microcontroller: DOIT ESP32 DEVKIT V1 (30 Pin)
 * Connections:     - Button:         D23
 *                  - OLED Display:   SDA (D21) / SCL (D22)   -- Note: Address is set to be 0x3C, change in definition if necessary.
 *                  - MAX30102:       SDA (D21) / SCL (D22)   -- Note: The MAX30102 shares the same library as MAX3010X.
 * Notes:           If flashing fails, try pressing BOOT Button during upload. Once flash succeeds, press reset button to restart the microcontroller.
 */

// Include required libraries
  #include <Wire.h>
  #include "MAX30105.h"
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include "heartRate.h"

// Declaration for Button
  #define BUTTON 23

// Declaration for OLED display. The pins for I2C are defined by the Wire-library. 
  #define SCREEN_WIDTH 128    // OLED display width, in pixels
  #define SCREEN_HEIGHT 64    // OLED display height, in pixels
  #define OLED_RESET    -1    // Reset Pin number, -1 for none
  #define SCREEN_ADDRESS 0x3C // OLED Display I2C address
  #define UNDEF         -1    // Custom undefined definition
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Declaration for the MAX30102 Sensor
  MAX30105 rateSensor;      // Declare heart rate sensor (The sensor used is MAX30102, but library is shared for MAX3010X)
  const byte RATE_SIZE = 5; // Increase this for more averaging
  byte rates[RATE_SIZE];    // Array of heart rates, used for heart rate averaging
  byte rate_index = 0;      // Current index location in the heart rates array
  long lastBeat = 0;        // Time at which the last beat occurred
  float instant_bpm;        // Instant calculation of heart rate
  int avg_bpm;              // Averaged calculation of heart rate
  bool zeros = true;        // True if the array has not filled up

// Function declaration
  void resetVariables();
  void errorLight();
  void displayText(bool next_line = true, char string[] = "", int text_size = 1, int cursor_x = UNDEF, int cursor_y = UNDEF, bool end_line = true, int display_color = SSD1306_WHITE);
  void displayNum(long num = UNDEF, int text_size = 2);

void setup() {
  //Initialize serial communication. Buad rate 115200.
  Serial.begin(115200);

  //Initialize LED and Push Button
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON, INPUT);
  
  // Allocate OLED Display with its screen address as defined. SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally.
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");

    // Stop program here if the allocation fails and display indication light on the ESP32
    while(true) {
      errorLight();
    }
  }

  // Initialize heart rate sensor. Use default I2C port, 400kHz speed
  if (!rateSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30105 was not found.");

    // Stop program here if the initiation fails and display indication light on the ESP32
    while(true) {
      errorLight();
    }
  }

  //Configure heart rate sensor settings
  rateSensor.setup();                    // Set to default settings
  rateSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low
  rateSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED
}

void loop() {
  // Set device to default state. Variables are zeroed, display IDLE text.
  resetVariables();
  display.clearDisplay();
  displayText(true, "IDLE...", 2, 0, 0);
  displayText();
  displayText(false, "Press button to start");

  // User to push button.
  while(digitalRead(BUTTON) == LOW);

  // Wait for the button to be released to prevent signal bouncing.
  while(digitalRead(BUTTON) == HIGH);

  // Continue reading heart rate until button is pressed again.
  while(digitalRead(BUTTON) == LOW) {
    // Obtain sensor reading
    long ir = rateSensor.getIR();

    // Check if something is on the sensor, and if there is a heart beat.
    if (checkForBeat(ir) && ir > 50000) {
      // Flash the onboard LED if a heart beat is detected.
      digitalWrite(LED_BUILTIN,HIGH);

      // Calculate time difference between current beat and last beat. Then calculate the instantaneous heart rate.
      long delta = millis() - lastBeat;
      instant_bpm = 60 / (delta / 1000.0);

      //set the current time as the last detected beat.
      lastBeat = millis();

      // If the heart rate is valid, store it in the reading array
      if (instant_bpm < 255 && instant_bpm > 20) {
        rates[rate_index++] = (byte)instant_bpm;

        // Move array index to the next one (overwrite once the array is filled)
        rate_index %= RATE_SIZE;

        // Once the array is filled, set zeros to false and start displaying averaged heart rate
        if (rate_index == 0) {
          zeros = false;
        }
        
        avg_bpm = 0;
        //Take average of readings if the array is filled
        if (!zeros) { 
          for (byte x = 0 ; x < RATE_SIZE ; x++) {
            avg_bpm += rates[x];
          }
          avg_bpm /= RATE_SIZE;
        }
      }
    }

    display.clearDisplay();
    
    // If a finger is not detected, show information to put finger on the sensor or exit the measurement.
    // If a finger is detected, show the current IR measurement and averaged heart rate information.
    if (ir < 50000) {
      displayText(true, "Place your index finger on the sensor with steady pressure.", 1, 0, 0);
      displayText();
      displayText(false, "Press button to exit.");
      resetVariables();
    }
    else {
      // Display IR reading
      displayText(true, "IR: ", 2, 0, 0, false);
      displayNum(ir);

      // Display heart rate reading
      displayText(true, "BPM: ", 2, UNDEF, UNDEF, false);
      // If there are no calculations yet, show a dash "-".
      if (avg_bpm == 0) {
        displayText(true, "-", 2, UNDEF, UNDEF);
      }
      else {
        displayNum(avg_bpm);
      }

      //Display information about the blue LED flash indicator.
      displayText();
      displayText(false, "Each blue LED flash indicates the detection of one heart beat.");
    }

    // Turn off the LED after 200 ms has expired since the last heart beat.
    if (lastBeat + 200 <= millis()) {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  
  // Wait for button to be released to prevent signal bouncing.
  while(digitalRead(BUTTON) == HIGH);
}

// Clear all variables and set to default state.
void resetVariables() {
  instant_bpm = 0;
  avg_bpm = 0;
  memset(rates, 0, sizeof(rates));
  zeros = true;
}

// Show the error LED indicator (Two short flashes.)
void errorLight() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

// Display a string of text on the OLED Display.
void displayText(bool next_line, char string[], int text_size, int cursor_x, int cursor_y, bool end_line, int display_color) {
  // Set the text color and text size. 
  display.setTextColor(display_color);
  display.setTextSize(text_size);

  // If the x & y location for the cursor is not the custom undefined number, set it to that location.
  if (cursor_x != UNDEF && cursor_y != UNDEF) {
    display.setCursor(cursor_x, cursor_y);
  }

  // If this string is the end of the line, print a line skip. Otherwise, just print the text.
  if (end_line) {
    display.println(string);
  }
  else {
    display.print(string);
  }

  // If there is no next line to print, display the buffer.
  if (!next_line) {
    display.display();
  }
}

// Display a number on the OLED display
void displayNum(long num, int text_size) {
  // Set text size.
  display.setTextSize(text_size);

  // If number is not a custom undefined number, print it.
  if (num != UNDEF) {
    display.println(num);
  }
}