#include <Arduino.h>
#include <CapacitiveSensor.h>

// val must be > than avg-value to trigger 1 tap
#define threshold 400

#define ButtonStairs 5   // ON-BOARD BUTTON INPUT
#define ButtonRoom 8     // EXTERNAL-BUTTON INPUT
#define ActivateStairs 2 // ON-BOARD MULTISWITCH INPUT
#define ActivateRoom 3   // ON-BOARD MULTISWITCH INPUT

#define OnTime A0 // ON-BOARD POTENTIOMETER (1) INPUT

#define Stairs A4 // RELAY (1) OUTPUT
#define Room A2   // RELAY (2) OUTPUT

#define CapSensorIn 6 // CAPSENSOR WITH EXTERNAL STAIR INPUT
#define CapSensor 10  // CAPSENSOR RESISTOR OTHER

// current avg value
int avgValue = 50;

// count time remaining in seconds for the lighting to stay on
float remainingTime = -5;

CapacitiveSensor sensor = CapacitiveSensor(CapSensor, CapSensorIn);

void startupsequence();

void setup()
{
  // Serial.begin(9600);

  pinMode(ButtonStairs, INPUT_PULLUP);
  pinMode(ButtonRoom, INPUT_PULLUP);
  pinMode(ActivateStairs, INPUT_PULLUP);
  pinMode(ActivateRoom, INPUT_PULLUP);

  pinMode(OnTime, INPUT);

  pinMode(Stairs, OUTPUT);
  pinMode(Room, OUTPUT);

  sensor.set_CS_AutocaL_Millis(4294967295);
  startupsequence();
}

void testStairs();
void testRoom();

void loop()
{
  if (digitalRead(ActivateStairs) == LOW) // ONBOARD SWITCH
    testStairs();
  else
    digitalWrite(Stairs, LOW);

  if (digitalRead(ActivateRoom) == LOW) // ONBOARD SWITCH
    testRoom();
  else
    digitalWrite(Room, LOW);

  delay(100);
}

void testRoom()
{
  if (digitalRead(ButtonRoom) == LOW) // EXTERNAL BUTTON
    digitalWrite(Room, HIGH);
  else
    digitalWrite(Room, LOW);
}

void testStairs()
{
  // read how long lamp should stay on
  float ontime = map(analogRead(OnTime), 0, 1023, 0, 30);

  // read cap value
  int val = sensor.capacitiveSensor(30);
  // avgValue = ((avgValue * 15) + val) >> 4;
  // avgValue = ((avgValue * 255) + val) >> 8;
  avgValue = ((avgValue * 63) + val) >> 6;
  // Serial.println(val);
  // Serial.println(avgValue);
  // Serial.println("");

  // check if value is greater threshold
  // digitalRead(ButtonStairs) == LOW)  triggers lighting directly if ONBOARD BUTTON is pressed
  if ((val > threshold + avgValue) || digitalRead(ButtonStairs) == LOW)
  {
    digitalWrite(Stairs, HIGH);
    remainingTime = ontime;
  }
  else
  {
    // only count if it wasnt already deactivated
    if (remainingTime != -5)
    {
      remainingTime -= 0.1;
      // deactivate lighting if time is up
      if (remainingTime <= 0)
      {
        digitalWrite(Stairs, LOW);
        remainingTime = -5;
      }
    }
  }
}

void startupsequence()
{
  digitalWrite(Stairs, HIGH);
  delay(300);
  digitalWrite(Room, HIGH);
  delay(500);
  digitalWrite(Stairs, LOW);
  delay(200);
  digitalWrite(Room, LOW);
  delay(200);
}