#include <SPI.h>

//Pins:
#define MUXS0 PB0
#define MUXS1 PB1
#define MUXS2 PB2
#define MUXS3 PB3
#define THERMISTORPIN PA7
#define CURRENTSENSORPIN PA6
#define RELAYPIN PD5

// BQ76952 Chip Select
#define CSPIN PB4

#define MAXCURRENT 90.0  // Maximum Current
#define MINCELL_VOLTAGE 45.0  // Minimum Voltage
#define MAXTHERMISTOR_TEMP 50.0  // Maximum Temperature
void setup() {

  pinMode(MUXS0, OUTPUT);
  pinMode(MUXS1, OUTPUT);
  pinMode(MUXS2, OUTPUT);
  pinMode(MUXS3, OUTPUT);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(THERMISTORPIN, INPUT);
  pinMode(CURRENTSENSORPIN, INPUT);
  pinMode(CSPIN, OUTPUT);

  SPI.begin();
  digitalWrite(CSPIN, HIGH);

  // Set relay off
  digitalWrite(RELAYPIN, LOW);

  Serial.begin(9600);
}

void loop() {
  //Check conditions
  bool thermistorCheck = checkThermistors();
  float current = readCurrent();
  float minCellVoltage = readCellVoltages();

  // Debugging
  //Serial.print("Thermistor Check: ");
  //Serial.println(thermistorCheck);
  //Serial.print("Current: ");
  //Serial.println(current);
  //Serial.print("Minimum Cell Voltage: ");
  //Serial.println(minCellVoltage);

  if (thermistorCheck && current < MAXCURRENT && minCellVoltage >= MINCELLVOLTAGE) {
    digitalWrite(RELAYPIN, HIGH); // Enable circuit
    Serial.println("Relay ON");
  } else {
    digitalWrite(RELAYPIN, LOW); // Disable circuit
    Serial.println("Relay OFF");
  }

  delay(1000);
}

bool checkThermistors() {
  for (int i = 0; i < 16; i++) { //MUX Iteration
    setMUXChannel(i);
    delay(10);
    int adcValue = analogRead(THERMISTORPIN);
    float temperature = calculateTemperature(adcValue);
    if (temperature > MAXTHERMISTORTEMP) {
      Serial.print(i);
      Serial.print(" Temperature: ");
      Serial.println(temperature);
      return false;
    }
  }
  return true;
}

float readCurrent() {
  int adcValue = analogRead(CURRENTSENSORPIN);
  float voltage = (adcValue / 1023.0) * 5.0; //Convert ADC value
  return (voltage / 4.5) * 100.0; //Convert voltage to current
}

float readCellVoltages() {
  digitalWrite(CSPIN, LOW); // Select BQ76952
  byte cellVoltageReg = 0x02;
  SPI.transfer(cellVoltageReg);
  byte highByte = SPI.transfer(0x00);
  byte lowByte = SPI.transfer(0x00);
  digitalWrite(CSPIN, HIGH);

  int rawValue = (highByte << 8) | lowByte;
  return rawValue * 0.001;
}

void setMUXChannel(int channel) {
  digitalWrite(MUXS0, channel & 0x01);
  digitalWrite(MUXS1, (channel >> 1) & 0x01);
  digitalWrite(MUXS2, (channel >> 2) & 0x01);
  digitalWrite(MUXS3, (channel >> 3) & 0x01);
}

float calculateTemperature(int adcValue) {
  // Calculate resistance
  float resistance = (10000.0 * adcValue) / (1023.0 - adcValue);

  //Equation from datasheet:
  float resistance = resistance / 10000.0;
  resistance = log(resistance);
  resistance /= 3950.0; 
  resistance += 1.0 / (25.0 + 273.15);
  resistance = 1.0 / resistance;
  resistance -= 273.15;
  return resistance;
}
