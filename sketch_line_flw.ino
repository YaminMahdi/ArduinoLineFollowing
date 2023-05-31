#define MOTOR_SPEED 170
#define leftMotorSpeed 80
#define rightMotorSpeed 80

#define black 500

// IR array
// int ir1 = A4, ir2 = A3, ir3 = A2, ir4 = A1, ir5 = A0;
int ir1 = A0, ir2 = A1, ir3 = A2, ir4 = A3, ir5 = A4;

// values
int v1 = 0, v2 = 0, v3 = 0, v4 = 0, v5 = 0, active = 0;

// Left motor int enableLeftMotor = 5, leftMotorPinNeg = 7, leftMotorPinPos = 6;
int enableLeftMotor = 10, leftMotorPinNeg = 9, leftMotorPinPos = 8;

// Right motor int enableRightMotor = 10, rightMotorPinNeg = 9, rightMotorPinPos = 8;
int enableRightMotor = 5, rightMotorPinNeg = 7, rightMotorPinPos = 6;

void setup()
{
  // The problem with TT gear motors is that, at very low pwm value it does not even rotate.
  // If we increase the PWM value then it rotates faster and our robot is not controlled in that speed and goes out of line.
  // For that we need to increase the frequency of analogWrite.
  // Below line is important to change the frequency of PWM signal on pin D5 and D6
  // Because of this, motor runs in controlled manner (lower speed) at high PWM value.
  // This sets frequecny as 7812.5 hz.
  //  TCCR0B = TCCR0B & B11111000 | B00000010 ;

  Serial.begin(9600);

  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPinNeg, OUTPUT);
  pinMode(rightMotorPinPos, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPinNeg, OUTPUT);
  pinMode(leftMotorPinPos, OUTPUT);

  //  pinMode(IR_SENSOR_RIGHT, INPUT);
  //  pinMode(IR_SENSOR_LEFT, INPUT);
}

void loop()
{

  //  delay(black0);
  v1 = analogRead(ir1);
  v2 = analogRead(ir2);
  v3 = analogRead(ir3);
  v4 = analogRead(ir4);
  v5 = analogRead(ir5);

  Serial.print("s1: ");
  Serial.print(v1);
  Serial.print("   s2: ");
  Serial.print(v2);
  Serial.print("   s3: ");
  Serial.print(v3);
  Serial.print("   s4: ");
  Serial.print(v4);
  Serial.print("   s5: ");
  Serial.println(v5);

  if (
      (v1 > black && v2 > black && v3 <= black && v4 > black && v5 > black) || (v1 > black && v2 <= black && v3 <= black && v4 <= black && v5 > black)
      // || (v1>black && v2>black && v3>black && v4>black && v5>black)
  )
  {
    analogWrite(enableRightMotor, rightMotorSpeed - 20);
    analogWrite(enableLeftMotor, leftMotorSpeed - 20);
    goForward();
    active = 0;
  }
  else if ( // left
      (v1 > black && v2 <= black && v3 <= black && v4 > black && v5 > black) ||
      (v1 > black && v2 <= black && v3 > black && v4 > black && v5 > black) ||
      (v1 <= black && v2 <= black && v3 > black && v4 > black && v5 > black) ||
      (v1 <= black && v2 > black && v3 > black && v4 > black && v5 > black))
  {
    analogWrite(enableRightMotor, rightMotorSpeed + 30);
    analogWrite(enableLeftMotor, leftMotorSpeed - 70);
    goForward();
    delay(100);
    // active = (active == 0) ? 1 : 0;
    active = 1;
  }
  else if (
      (v1 <= black && v2 <= black && v3 <= black && v4 > black && v5 > black) ||
      (v1 <= black && v2 <= black && v3 <= black && v4 <= black && v5 > black)

  )
  {
    analogWrite(enableRightMotor, rightMotorSpeed);
    analogWrite(enableLeftMotor, leftMotorSpeed);
    goForward();
    delay(150);
    analogWrite(enableRightMotor, rightMotorSpeed + 70);
    analogWrite(enableLeftMotor, leftMotorSpeed + 30);
    goLeft90();
    delay(400);
    active = 1;
  }
  else if ( // right
      (v1 > black && v2 > black && v3 <= black && v4 <= black && v5 > black) ||
      (v1 > black && v2 > black && v3 > black && v4 <= black && v5 > black) ||
      (v1 > black && v2 > black && v3 > black && v4 <= black && v5 <= black) ||
      (v1 > black && v2 > black && v3 > black && v4 > black && v5 <= black))
  {
    analogWrite(enableRightMotor, rightMotorSpeed - 70);
    analogWrite(enableLeftMotor, leftMotorSpeed + 30);
    goForward();
    delay(100);
    active = 1;
  }
  else if (
      (v1 > black && v2 > black && v3 <= black && v4 <= black && v5 <= black) ||
      (v1 > black && v2 <= black && v3 <= black && v4 <= black && v5 <= black))
  {
    analogWrite(enableRightMotor, rightMotorSpeed);
    analogWrite(enableLeftMotor, leftMotorSpeed);
    goForward();
    delay(150);
    analogWrite(enableRightMotor, rightMotorSpeed + 30);
    analogWrite(enableLeftMotor, leftMotorSpeed + 70);
    goRight90();
    delay(400);
    active = 1;
  }
  else if (
      (v1 <= black && v2 <= black && v3 <= black && v4 <= black && v5 <= black) ||
      (v1 > black && v2 <= black && v3 > black && v4 <= black && v5 > black) ||
      (v1 <= black && v2 > black && v3 <= black && v4 > black && v5 <= black))
  {
    analogWrite(enableRightMotor, 0);
    analogWrite(enableLeftMotor, 0);
    stop();
    active = 0;
  }
  else if (
      (v1 > black && v2 > black && v3 > black && v4 > black && v5 > black) && active == 1)
  {
    analogWrite(enableRightMotor, rightMotorSpeed - 40);
    analogWrite(enableLeftMotor, leftMotorSpeed - 20);
    goBackward();
  }
  else
  {
    analogWrite(enableRightMotor, 0);
    analogWrite(enableLeftMotor, 0);
    stop();
    active = 0;
  }

  //   goForward();
  //   delay(450);

  //   goBackward();
  //   delay(1000);

  //   goLeft();
  //   delay(400);

  //   goForward();
  //   delay(450);

  //   goBackward();
  //   delay(1000);

  // goRight();
  // delay(400);

  //   goBackward();
  //   delay(750);

  //   goLeft90();
  //   delay(1000);

  //   goRight90();
  //   delay(1000);

  // analogWrite(enableRightMotor, 60);
  // analogWrite(enableLeftMotor, 50);
}

void goForward()
{
  digitalWrite(rightMotorPinNeg, LOW);
  digitalWrite(rightMotorPinPos, HIGH);

  digitalWrite(leftMotorPinNeg, LOW);
  digitalWrite(leftMotorPinPos, HIGH);
}

void goBackward()
{
  digitalWrite(rightMotorPinNeg, HIGH);
  digitalWrite(rightMotorPinPos, LOW);

  digitalWrite(leftMotorPinNeg, HIGH);
  digitalWrite(leftMotorPinPos, LOW);
}

void stop()
{
  digitalWrite(rightMotorPinNeg, LOW);
  digitalWrite(rightMotorPinPos, LOW);

  digitalWrite(leftMotorPinNeg, LOW);
  digitalWrite(leftMotorPinPos, LOW);
}

void goRight()
{
  digitalWrite(rightMotorPinNeg, LOW);
  digitalWrite(rightMotorPinPos, LOW);

  digitalWrite(leftMotorPinNeg, LOW);
  digitalWrite(leftMotorPinPos, HIGH);
}

void goRight90()
{
  digitalWrite(rightMotorPinNeg, HIGH);
  digitalWrite(rightMotorPinPos, LOW);

  digitalWrite(leftMotorPinNeg, LOW);
  digitalWrite(leftMotorPinPos, HIGH);
}

void goLeft()
{
  digitalWrite(leftMotorPinNeg, LOW);
  digitalWrite(leftMotorPinPos, LOW);

  digitalWrite(rightMotorPinNeg, LOW);
  digitalWrite(rightMotorPinPos, HIGH);
}

void goLeft90()
{
  digitalWrite(leftMotorPinNeg, HIGH);
  digitalWrite(leftMotorPinPos, LOW);

  digitalWrite(rightMotorPinNeg, LOW);
  digitalWrite(rightMotorPinPos, HIGH);
}
