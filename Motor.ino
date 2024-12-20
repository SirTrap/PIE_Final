int ra1 = 0;
int ra2 = 230;
int A = 2;

// PID
float kp = 3.2;  // P gain
float ki = 0.0;  // I gain
float kd = 0.4;  // Dgain

float pidError = 0;
float pidPreviousError = 0;
float pidIntegral = 0;

void setup() {
  pinMode(A1, INPUT); // Speed for Motor1
  pinMode(A2, INPUT); // Speed for Motor2
  pinMode(A0, OUTPUT); // Motor 1

  pinMode(2, OUTPUT); // Motor 1 direction
  pinMode(3, OUTPUT); // Motor 2 direction
  pinMode(7, OUTPUT); // Brakes Output

  pinMode(4, INPUT); // Brake Input
  pinMode(5, INPUT); // Direction Input 1
  pinMode(6, INPUT); // Direction Input 2

  Serial.begin(9600);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
}

void loop() {
  // Read speed from motor controller
  int speed1 = analogRead(A1);
  int speed2 = analogRead(A2); 

  // Calculate error
  pidError = speed1 - speed2;

  float pidProportional = kp * pidError;
  pidIntegral += ki * pidError;
  float pidDerivative = kd * (pidError - pidPreviousError);

  // PID output
  float pidOutput = pidProportional + pidIntegral + pidDerivative;

  if (pidOutput > 255) pidOutput = 255;
  if (pidOutput < -255) pidOutput = -255;

  //Update error
  pidPreviousError = pidError;

  // Debugging output
  //Serial.print("Speed1: ");
  //Serial.print(speed1);
  //Serial.print(" Speed2: ");
  //Serial.print(speed2);
  //Serial.print(" PID Output: ");
  //Serial.println(pidOutput);

  if (digitalRead(4) == LOW) {
    digitalWrite(2, HIGH);
  } else if (digitalRead(4) == HIGH) {
    digitalWrite(2, LOW);
  }

  if (digitalRead(5) == HIGH) {
    digitalWrite(3, HIGH);
    digitalWrite(7, HIGH);
    ra1 = pidOutput + 255 * analogRead(A1) / 1023 / A + 65;
    if (ra1 > 255) {
      ra1 = 255;
    }
    analogWrite(A0, ra1);
  } else if (digitalRead(6) == HIGH) {
    digitalWrite(3, LOW);
    digitalWrite(7, LOW);
    ra1 = pidOutput + 255 * analogRead(A1) / 1023 / A + 65;
    if (ra1 > 255) {
      ra1 = 255;
    }
    analogWrite(A0, ra1);
  }
}