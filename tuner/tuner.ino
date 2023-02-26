#include "arduinoFFT.h"
#include "math.h"
#include <LiquidCrystal.h>

const int rs = 17, en = 18, d4 = 19, d5 = 20, d6 = 21, d7 = 22;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Declaring the pins that I use for LCD

const int button_decided = 15; // Pressed button if the decision is made
const int button_forward = 23; // Pressed button for deciding frequency configuration: Go forward
const int button_backward = 16; // Pressed button for deciding frequency configuration: Go backward

/* Frequency configurations that we can achieve */
double freq_configurations[7][7] = {
  {0, 82.4, 110, 146.8, 196, 247, 329.6}, // E standard
  {0, 82.4, 123.5, 164.8, 207.6, 247, 329.6}, // Open E
  {0, 65.4, 98, 130.8, 196, 261.3, 329.6}, // Open C
  {0, 73.4, 110, 146.8, 185, 220, 293.6}, // Open D
  {0, 73.4, 110, 146.8, 196, 247, 329.6}, // Drop D
  {0, 73.4, 110, 146.8, 196, 220, 293.7}, // DADGAD
  {0, 77.8, 103.8, 138.6, 185, 233.1, 311.13}  // Half-Step Down
};

/* Names of the frequency configurations */
char freq_config_names[7][15] = {"E Standard", "Open E", "Open C", "Open D",
                                 "Drop D", "DADGAD", "Half-Step Down"
                                };


char string_names[7][11] = {"", "1st String", "2nd String", "3rd String", "4th String",
                            "5th String", "6th String"
                           };

String old_frequency = "...";

int location = 0, string_location = 1; // To go back and forth in configurations, initially 0
int len, space, num, str, selection = 0; // Variables about LCD display
int buttonState_f = 0, buttonState_b = 0, buttonState_d = 0; // Current state of the buttons
int lastButtonState_f = 0, lastButtonState_b = 0, lastButtonState_d = 0; // Previous state of the buttons

// MOTOR VARIABLES

/* Declaring motor direction digital pin array, 0 is empty */
const int motor_dir[] = {0, 12, 11, 8, 1, 2, 7,};
/* Declaring motor rotation speed PWM pin array, 0 is empty */
const int motor_speed[] = {0, 10, 9, 4, 3, 5, 6,};

/* Declaring variables which keep speed and direction */
bool sign[7];
int analog_speed[7];

// FREQUENCY VARIABLES

/* Creating FFT object */
arduinoFFT FFT = arduinoFFT();

/* Declaring FFT Variables */
unsigned int sampling_period_us;
unsigned long microseconds;
const uint16_t samples = 2048; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 883;
int i, j;
int duration[7];
double del_j, a, b, c;
double temp, step_size, vReal[samples], vImag[samples];

/* Declaring frequency variables */
double correctFrequency[7]; // 0 is empty
double frequency[7]; // 0 is empty
double freq_diff[7]; // 0 is empty
double x; // Temporary variable which carries absolute value of freq_diff
double freq_tolerance = 0.9;

void LCD_select() {
  lcd.setCursor(0, 0); // Show at top left
  lcd.print("Welcome to tuner");
  lcd.setCursor(1, 1);
  lcd.print("Select Tuning!");
  delay(5000);
  Print_Config_Name();
  while (1) { // Desired frequency selection
    // Reading the current values of buttons
    buttonState_f = digitalRead(button_forward);
    buttonState_b = digitalRead(button_backward);
    buttonState_d = digitalRead(button_decided);

    // If there is a change in the state of the button
    if (buttonState_f != lastButtonState_f) {
      delay(30);
      // and it is HIGH, go forward in the frequency configuration array
      if (buttonState_f == HIGH) {
        location++;
        if (location == 7) location = 0;
        Print_Config_Name();
      }
    }

    // If there is a change in the state of the button
    if (buttonState_b != lastButtonState_b) {
      delay(30);
      // and it is HIGH, go backward in the frequency configuration array
      if (buttonState_b == HIGH) {
        location--;
        if (location == -1) location = 6;
        Print_Config_Name();
      }
    }

    // Updating last states with the current of previous time to detect if there is a changement
    lastButtonState_f = buttonState_f;
    lastButtonState_b = buttonState_b;


    // If "decision is made" button is pressed, update the desired frequency. We are ready to tune!
    if (buttonState_d == HIGH) {
      lcd.clear();
      lcd.setCursor(0, 0); // Show at top left
      lcd.print("Target tuning is");
      len = strlen(freq_config_names[location]);
      space = (16 - len) / 2;
      lcd.setCursor(space, 1); // Show at bottom left
      lcd.print(freq_config_names[location]);
      for (i = 0; i < 7; i++) {
        correctFrequency[i] = freq_configurations[location][i];
      }
      selection = 1;
      delay(3000);
      break;
    }
  }
}
void Print_Config_Name() {
  lcd.clear();
  len = strlen(freq_config_names[location]);
  space = (16 - len) / 2;
  lcd.setCursor(space, 0);
  lcd.print(freq_config_names[location]);
  lcd.setCursor(0, 1);
  lcd.print("<-            ->");
}
void printString() {
  lcd.clear();
  len = strlen(string_names[string_location]);
  space = (16 - len) / 2;
  lcd.setCursor(space, 0);
  lcd.print(string_names[string_location]);

  String frequency_goal;
  if (frequency[string_location] > 0) {
    frequency_goal = String(frequency[string_location]) + "->" + String(correctFrequency[string_location]);
    old_frequency = String(frequency[string_location]);
  }
  else {
    frequency_goal = old_frequency + "->" + String(correctFrequency[string_location]);
  }
  len = frequency_goal.length();
  space = (16 - len) / 2;
  lcd.setCursor(space, 1);
  lcd.print(frequency_goal);
  lcd.setCursor(space, 1);
}
void getFrequency() {
  //SAMPLING:
  microseconds = micros(); //Overflows after around 70 minutes!
  for (i = 0; i < samples; i++) {
    if (i < samples / 4) {
      vReal[i] = analogRead(A0);
      vImag[i] = 0;
      while ((micros() - microseconds) < sampling_period_us) {}
      microseconds = microseconds + sampling_period_us;
    }
    else {
      vReal[i] = 0;
      vImag[i] = 0;
    }
  }

  //FFT:
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */


  //RESULT:
  frequency[1] = 0;
  frequency[2] = 0;
  frequency[3] = 0;
  frequency[4] = 0;
  frequency[5] = 0;
  frequency[6] = 0;

  int interval_value = 35;

  int threshold_value;
  if (string_location < 4 && string_location > 0) {
    threshold_value = 2500;
  }
  else if (string_location == 4) {
    threshold_value = 1000;
  }
  else if (string_location == 5) {
    threshold_value = 500;
  }
  else if (string_location == 6) {
    threshold_value = 500;
  }
  double lower = correctFrequency[string_location] - interval_value;
  double upper = correctFrequency[string_location] + interval_value;

  frequencyAnalyzer(string_location, threshold_value, lower, upper);
}
void frequencyAnalyzer (int string, int temp, double lower_bound, double upper_bound) {
  for (i = round(lower_bound / step_size); i < round(upper_bound / step_size); i++) {
    if (vReal[i] > temp) {
      temp = vReal[i];
      frequency[string] = i * 1.0 * step_size;
    }
  }
}
void printFrequency() {
  if (frequency[string_location] != 0) {
    Serial.print("\n FREQUENCY: ");
    Serial.print(frequency[string_location]);
  }
}
void setDirection() {
  analog_speed[string_location] = 0;
  freq_diff[string_location] = 0;
  freq_diff[string_location] = frequency[string_location] - correctFrequency[string_location];
  if (freq_diff[string_location] > 0) {
    sign[string_location] = 0; //Unstrech the string.
  }
  else {
    sign[string_location] = 1; //Strech the string.
  }
  x = abs(freq_diff[string_location]);
  if (frequency[string_location] == 0) {
    analog_speed[string_location] = 0;
  }
  else if ( x > freq_tolerance) {
    analog_speed[string_location] = 255;
  }
  else if ( x <= freq_tolerance) {
    analog_speed[string_location] = 0;
    doneTuning();
  }
}
void setDuration() {
  duration[string_location] = 0;
  if (frequency[string_location] != 0) {
    double y = abs(freq_diff[string_location]);
    duration[string_location] = y / 2.3 * 100;
    if (duration[string_location] > 400) {
      duration[string_location] = 400;
    }
    //    if (y > 4.9) {
    //      duration[string_location] = 400;
    //    }
    //    else if (y < 4.9 && y > 4.2) {
    //      duration[string_location] = 350;
    //    }
    //    else if (y < 4.2 && y > 3.6) {
    //      duration[string_location] = 300;
    //    }
    //    else if (y < 3.6 && y > 3) {
    //      duration[string_location] = 250;
    //    }
    //    else if (y < 3 && y > 2.3) {
    //      duration[string_location] = 200;
    //    }
    //    else if (y < 2.3 && y > 1.65) {
    //      duration[string_location] = 150;
    //    }
    //    else if (y < 1.65 && y > freq_tolerance) {
    //      duration[string_location] = 100;
    //    }
  }
}
void writeToMotor() {
  if (sign[string_location] == 0) {
    digitalWrite(motor_dir[string_location], HIGH); //Unstrech the string.
  }
  else if (sign[string_location] == 1) {
    digitalWrite(motor_dir[string_location], LOW); //Strech the string.
  }
  analogWrite(motor_speed[string_location], analog_speed[string_location]);
}
void doneTuning() {
  string_location++;
  if (string_location < 7) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("String is done,");
    lcd.setCursor(0, 1);
    lcd.print("Play next!");
    old_frequency = "...";
    delay(4000);

  }
  else {
    string_location = 1;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Tuning is done!");
    old_frequency = "...";
    while (1) {
      buttonState_d = digitalRead(button_decided);
      if (buttonState_d == HIGH) {
        Serial.println("lan");
        selection = 1;
        delay(3000);
        break;
      }
    }
    selection = 0;
  }
}
void setup() {
  /* Setting pin modes */
  for (i = 1; i < 7; i++) {
    pinMode(motor_dir[i], OUTPUT);
    pinMode(motor_speed[1], OUTPUT);
  }

  /* Initially setting the motors off */
  for (i = 1; i < 7; i++) {
    analogWrite(motor_speed[i], 0);
  }

  /* Creating the Serial Communication Port */
  Serial.begin(9600);

  /* Sampling Period and Step Size of FFT */
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  step_size = samplingFrequency / samples;

  /* Setting LCD and Push Buttons */
  digitalWrite(en, HIGH);
  pinMode(button_decided, INPUT);
  pinMode(button_forward, INPUT);
  pinMode(button_backward, INPUT);
  lcd.begin(16, 2); // Declaring LCD
}
void loop() {
  if (selection == 0) {
    LCD_select();
  }
  printString();
  getFrequency();
  printFrequency();
  setDirection();
  setDuration();
  writeToMotor();
  Serial.println("");
  Serial.println(duration[string_location]);
  delay(duration[string_location]);
  analogWrite(motor_speed[string_location], 0);
}
