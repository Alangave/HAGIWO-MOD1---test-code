/*
HAGIWO MOD1 RandomCV Ver1.0
Periodic random CV sequencer.

--Pin assign---
POT1  A0  Step length 3,4,5,8,16,32
POT2  A1  output level
POT3  A2  Trigger probability
F1    D17  Clock in
F2    D9  Random value update
F3    D10  CV output
F4    D11 Trigger output
BUTTON    Random value update
LED       CV output
EEPROM    N/A
*/
#include <EEPROM.h>
#define TABLE_SIZE 1024
#define Brightness 255//Adjust according to the luminance of the LEDs used. Value range is 0-255

float lfoFrequency = 0.1;
int waveHeight = 255;
float waveIndex = 0;
long outputValue = 0;
int outputLed = 0;
int freqRange = 10;
int buttonOn = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

bool lastButtonState = HIGH;
unsigned long buttonPreviousMillis = 0;
const unsigned long debounceDelay = 50;

// button & led pin assigment
const int ledPin = 3;
const int buttonPin = 4;

// pot assignment
const int pot1 = A0;
const int pot2 = A1;
const int pot3 = A2;

// Jacks digital in/out assignment
const int digitJackF1 = 17; 
const int digitJackF2 = 9; 
const int digitJackF3 = 10;
const int digitJackF4 = 11;

// analog in assignment
const int analogInJackF1 = A3;
const int analogInJackF2 = A4;
const int analogInJackF3 = A5;

const byte PROGMEM SinTable[TABLE_SIZE] = {
  127, 128, 129, 129, 130, 131, 132, 132, 133, 134, 135, 136, 136, 137, 138, 139, 139, 140, 141, 142, 143, 143, 144, 145, 146, 146, 147, 148, 149, 150, 150, 151, 152, 153, 153, 154, 155, 156, 156, 157, 158, 159, 159, 160, 161, 162, 163, 163, 164, 165, 166, 166, 167, 168, 168, 169, 170, 171, 171, 172, 173, 174, 174, 175, 176, 177, 177, 178, 179, 179, 180, 181, 182, 182, 183, 184, 184, 185, 186, 186, 187, 188, 188, 189, 190, 191, 191, 192, 193, 193, 194, 195, 195, 196, 197, 197, 198, 198, 199, 200, 200, 201, 202, 202, 203, 204, 204, 205, 205, 206, 207, 207, 208, 208, 209, 210, 210, 211, 211, 212, 213, 213, 214, 214, 215, 215, 216, 217, 217, 218, 218, 219, 219, 220, 220, 221, 221, 222, 223, 223, 224, 224, 225, 225, 226, 226, 227, 227, 228, 228, 228, 229, 229, 230, 230, 231, 231, 232, 232, 233, 233, 233, 234, 234, 235, 235, 236, 236, 236, 237, 237, 238, 238, 238, 239, 239, 239, 240, 240, 241, 241, 241, 242, 242, 242, 243, 243, 243, 244, 244, 244, 244, 245, 245, 245, 246, 246, 246, 247, 247, 247, 247, 248, 248, 248, 248, 249, 249, 249, 249, 249, 250, 250, 250, 250, 250, 251, 251, 251, 251, 251, 252, 252, 252, 252, 252, 252, 252, 253, 253, 253, 253, 253, 253, 253, 253, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 255, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 253, 253, 253, 253, 253, 253, 253, 253, 252, 252, 252, 252, 252, 252, 252, 251, 251, 251, 251, 251, 250, 250, 250, 250, 250, 249, 249, 249, 249, 249, 248, 248, 248, 248, 247, 247, 247, 247, 246, 246, 246, 245, 245, 245, 244, 244, 244, 244, 243, 243, 243, 242, 242, 242, 241, 241, 241, 240, 240, 239, 239, 239, 238, 238, 238, 237, 237, 236, 236, 236, 235, 235, 234, 234, 233, 233, 233, 232, 232, 231, 231, 230, 230, 229, 229, 228, 228, 228, 227, 227, 226, 226, 225, 225, 224, 224, 223, 223, 222, 221, 221, 220, 220, 219, 219, 218, 218, 217, 217, 216, 215, 215, 214, 214, 213, 213, 212, 211, 211, 210, 210, 209, 208, 208, 207, 207, 206, 205, 205, 204, 204, 203, 202, 202, 201, 200, 200, 199, 198, 198, 197, 197, 196, 195, 195, 194, 193, 193, 192, 191, 191, 190, 189, 188, 188, 187, 186, 186, 185, 184, 184, 183, 182, 182, 181, 180, 179, 179, 178, 177, 177, 176, 175, 174, 174, 173, 172, 171, 171, 170, 169, 168, 168, 167, 166, 166, 165, 164, 163, 163, 162, 161, 160, 159, 159, 158, 157, 156, 156, 155, 154, 153, 153, 152, 151, 150, 150, 149, 148, 147, 146, 146, 145, 144, 143, 143, 142, 141, 140, 139, 139, 138, 137, 136, 136, 135, 134, 133, 132, 132, 131, 130, 129, 129, 128, 127, 126, 125, 125, 124, 123, 122, 122, 121, 120, 119, 118, 118, 117, 116, 115, 115, 114, 113, 112, 111, 111, 110, 109, 108, 108, 107, 106, 105, 104, 104, 103, 102, 101, 101, 100, 99, 98, 98, 97, 96, 95, 95, 94, 93, 92, 91, 91, 90, 89, 88, 88, 87, 86, 86, 85, 84, 83, 83, 82, 81, 80, 80, 79, 78, 77, 77, 76, 75, 75, 74, 73, 72, 72, 71, 70, 70, 69, 68, 68, 67, 66, 66, 65, 64, 63, 63, 62, 61, 61, 60, 59, 59, 58, 57, 57, 56, 56, 55, 54, 54, 53, 52, 52, 51, 50, 50, 49, 49, 48, 47, 47, 46, 46, 45, 44, 44, 43, 43, 42, 41, 41, 40, 40, 39, 39, 38, 37, 37, 36, 36, 35, 35, 34, 34, 33, 33, 32, 31, 31, 30, 30, 29, 29, 28, 28, 27, 27, 26, 26, 26, 25, 25, 24, 24, 23, 23, 22, 22, 21, 21, 21, 20, 20, 19, 19, 18, 18, 18, 17, 17, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 18, 18, 18, 19, 19, 20, 20, 21, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38, 39, 39, 40, 40, 41, 41, 42, 43, 43, 44, 44, 45, 46, 46, 47, 47, 48, 49, 49, 50, 50, 51, 52, 52, 53, 54, 54, 55, 56, 56, 57, 57, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 72, 72, 73, 74, 75, 75, 76, 77, 77, 78, 79, 80, 80, 81, 82, 83, 83, 84, 85, 86, 86, 87, 88, 88, 89, 90, 91, 91, 92, 93, 94, 95, 95, 96, 97, 98, 98, 99, 100, 101, 101, 102, 103, 104, 104, 105, 106, 107, 108, 108, 109, 110, 111, 111, 112, 113, 114, 115, 115, 116, 117, 118, 118, 119, 120, 121, 122, 122, 123, 124, 125, 125, 126
};

char output[100]; // Buffer for serial output
char result[11]; // 10 caracters + for EOL
void oscillo(int value) {

  // Calculer la position du caractère '*'
  int position = value / 100;

  // Initialiser la chaîne avec des espaces
  for (int i = 0; i < position; i++) {
    result[i] = '=';
  }

  // Placer le caractère '*' à la position calculée
  result[position] = '0';
  // Remplir le reste de la chaîne avec des espaces
  for (int i = position + 1; i < 10; i++) {
    result[i] = ' ';
  }

  // Ajouter le caractère de fin de chaîne
  result[10] = '\0';
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(digitJackF4, INPUT_PULLUP);
  pinMode(digitJackF4, INPUT_PULLUP);
  pinMode(digitJackF4, INPUT_PULLUP);
  pinMode(digitJackF4, OUTPUT);

  freqRange = EEPROM.read(0);

  TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2B1);  //fast PWM 62.5kHz setting
  TCCR2B = (1 << CS20);                                  //fast PWM 62.5kHz setting
  TCCR1A = (1 << WGM11) | (1 << COM1A1);                 //fast PWM 62.5kHz setting
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);    //fast PWM 62.5kHz setting
  ICR1 = 255;                                            //fast PWM 62.5kHz setting

  //Serial

  Serial.begin(9600);
  Serial.println("startup...");

}

void loop() {

  currentMillis = millis();

  // Read the current state of the button
  int reading = digitalRead(buttonPin);
  if (currentMillis - buttonPreviousMillis > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      buttonOn = (buttonOn == 1) ? 0 : 1;
      buttonPreviousMillis = currentMillis;
    }
  }
  lastButtonState = reading;

  int potValueA0 = analogRead(pot1);  //frequency
  lfoFrequency = min(potValueA0, 1023) * 0.0015 * freqRange;


  if (currentMillis - previousMillis >= 1) {
    previousMillis = currentMillis;

    outputValue = pgm_read_byte(&SinTable[(int)waveIndex]) * waveHeight / 255;

    analogWrite(digitJackF4, outputValue);  //output LFO waveform
    analogWrite(ledPin, (byte)outputValue * Brightness /255);  //output LED


    waveIndex = waveIndex + lfoFrequency + 0.01;
    if (waveIndex >= TABLE_SIZE) {
      waveIndex -= TABLE_SIZE;
    }

    // Serial print values
    sprintf(output, "pot1: %4d | pot2: %4d pot3: %4d | button: %1d | led: %4d", analogRead(pot1),analogRead(pot2), analogRead(pot3), buttonOn, outputValue );
    Serial.print(output);
    Serial.print("| F1 ");
    oscillo(analogRead(analogInJackF1));
    Serial.print(result);
    Serial.print("| F2 ");
    oscillo(analogRead(analogInJackF2));
    Serial.print(result);
    Serial.print("| F3 ");
    oscillo(analogRead(analogInJackF3));
    Serial.println(result);

  };

}


