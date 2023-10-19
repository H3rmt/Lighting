#include <Arduino.h>
#include <CapacitiveSensor.h>

#define debugg

// val must be > than avg-value to trigger 1 tap
#define threshold 400

#define ButtonStairs 5   // ON-BOARD BUTTON INPUT
#define ButtonRoom 8     // EXTERNAL-BUTTON INPUT
#define ActivateStairs 2 // ON-BOARD MULTISWITCH INPUT
#define ActivateRoom 3   // ON-BOARD MULTISWITCH INPUT

#define OnTime A0               // ON-BOARD POTENTIOMETER (1) INPUT
#define LightSensor A5          // EXTERNAL PHOTOCELL INPUT
#define LightSensorThreshold A1 // ON-BOARD POTENTIOMETER (2) INPUT

#define Stairs A4 // RELAY (1) OUTPUT
#define Room A2   // RELAY (2) OUTPUT

#define CapSensorIn 6 // CAPSENSOR WITH EXTERNAL STAIR INPUT
#define CapSensor 10  // CAPSENSOR RESISTOR OTHER

// current avg value
int avgValue;

// count time remaining in seconds for the lighting to stay on
float remainingTime = 0;

CapacitiveSensor sensor = CapacitiveSensor(CapSensor, CapSensorIn);

void startupsequence();

void setup()
{
#ifdef debug
  Serial.begin(9600);
#endif

  pinMode(ButtonStairs, INPUT_PULLUP);
  pinMode(ButtonRoom, INPUT_PULLUP);
  pinMode(ActivateStairs, INPUT_PULLUP);
  pinMode(ActivateRoom, INPUT_PULLUP);

  pinMode(OnTime, INPUT);
  pinMode(LightSensor, INPUT);
  pinMode(LightSensorThreshold, INPUT);

  pinMode(Stairs, OUTPUT);
  pinMode(Room, OUTPUT);

#ifdef debug
  Serial.println("started");
#endif

  sensor.set_CS_AutocaL_Millis(4294967295);
  avgValue = 50;
  startupsequence();
}

void testStairs(boolean);
void testRoom();

int count = 0;

void loop()
{
  if (digitalRead(ActivateStairs) == LOW)
  // ONBOARD SWITCH
#ifdef debug
    if (count % 5 == 0)
    {
      testStairs(true);
      Serial.println("\n");
    }
    else
    {
      testStairs(false);
    }
#else
    testStairs(false);
#endif

  else
    digitalWrite(Stairs, LOW);

  if (digitalRead(ActivateRoom) == LOW) // ONBOARD SWITCH
    testRoom();
  else
    digitalWrite(Room, LOW);
  count++;

  delay(100);
}

void testRoom()
{
  if (digitalRead(ButtonRoom) == LOW)
  { // EXTERNAL BUTTON
    digitalWrite(Room, HIGH);
  }
  else
  {
    digitalWrite(Room, LOW);
  }
}

void testStairs(boolean deb)
{
  // read how long lamp should stay on
  float ontime = map(analogRead(OnTime), 0, 1023, 0, 30);

  // min value for photocell to activate the stairs light
  float lsThreshold = 1023 - map(analogRead(LightSensorThreshold), 0, 1023, 0, 255);

  // read cap value
  int val = sensor.capacitiveSensor(30);
  avgValue = ((avgValue * 15) + val) >> 4;

#ifdef debug
  if (deb)
  {
    Serial.print("lamp ontime:");
    Serial.println(ontime);
    Serial.print("value: ");
    Serial.println(val);
    Serial.print("avgValue: ");
    Serial.println(avgValue);
    Serial.print("missing value: ");
    Serial.println(avgValue + threshold - val);
    // Serial.print("light Sensor: ");
    // Serial.println(analogRead(LightSensor));
    // Serial.print("light Sensor Threshold: ");
    // Serial.println(lsThreshold);
  }
#endif
  // check if value is greater threshold
  // digitalRead(ButtonStairs) == LOW)  triggers lighting directly if ONBOARD BUTTON is pressed
  if ((val > threshold + avgValue && analogRead(LightSensor) > lsThreshold) || digitalRead(ButtonStairs) == LOW)
  {
    digitalWrite(Stairs, HIGH);
#ifdef debug
    if (deb)
    {
      Serial.print("activate with value:");
    }
#endif
    remainingTime = ontime;
  }
  else
  {

    // only count if it wasnt deactivated before
    if (remainingTime != -5)
    {
      remainingTime -= 0.1;
#ifdef debug
      if (deb)
      {
        Serial.print("time remaining:");
        Serial.println(remainingTime);
      }
#endif
      // deactivate lighting if time is up
      if (remainingTime <= 0)
      {
        digitalWrite(Stairs, LOW);
#ifdef debug
        if (deb)
        {
          Serial.println("deactivate");
        }
#endif
        remainingTime = -5;
      }
    }
#ifdef debug
    else
    {
      if (deb)
      {
        Serial.println("no change");
      }
    }
#endif
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