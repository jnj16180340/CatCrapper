#include <RTClib.h>
#include "LowPower.h"

// HUGE TODO!!!!!!
// "The charge circuit is programmed to charge the battery at 500mA, so, to be safe, the battery should be no smaller than 500mAH in capacity."

#define LED_PIN_BLUE 4
#define LED_PIN_GREEN 5
#define BUTTON_PIN 3  // 2,3 are the interrupts
// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 2

#define TIMESET_PIN_1 A0
#define TIMESET_PIN_2 A1
#define TIMESET_PIN_4 A2
#define TIMESET_PIN_8 A3
unsigned int timer_period = 0;

// 1-arg constructor is secnds only
//#define SLEEP_PERIOD 12
//#define SLEEP_PERIOD 2
//#define SLEEP_PERIOD TimeSpan(4)
// TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds);

// NB: This STILL needs TimeSpan to be set to not seconds.
//#define SLEEP_UNIT DS3231_A1_Second
//#define SLEEP_PERIOD TimeSpan(0, 0, 0, 10)
//#define SLEEP_PERIOD TimeSpan(10)

//#define SLEEP_UNIT DS3231_A1_Minute
//#define SLEEP_PERIOD TimeSpan(0, 0, 20, 0)

#define SLEEP_UNIT DS3231_A1_Hour
//#define SLEEP_PERIOD TimeSpan(0, 9, 0, 0)
TimeSpan SLEEP_PERIOD;

RTC_DS3231 rtc;

int state = HIGH;
// the setup function runs once when you press reset or power the board
// SDA: 4 ANALOG!!
// SCL: 5 ANALOG!!

volatile bool PLEASE_SET_ALARM = false;
volatile bool IN_ALARM = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN_BLUE, OUTPUT);   // LED PIN
  pinMode(LED_PIN_GREEN, OUTPUT);  // LED PIN

  pinMode(TIMESET_PIN_1, INPUT_PULLUP);
  pinMode(TIMESET_PIN_2, INPUT_PULLUP);
  pinMode(TIMESET_PIN_4, INPUT_PULLUP);
  pinMode(TIMESET_PIN_8, INPUT_PULLUP);

  if(!digitalRead(TIMESET_PIN_1)){timer_period += 1;}
  if(!digitalRead(TIMESET_PIN_2)){timer_period += 2;}
  if(!digitalRead(TIMESET_PIN_4)){timer_period += 4;}
  if(!digitalRead(TIMESET_PIN_8)){timer_period += 8;}
  

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // BUTTON
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButton, FALLING);

  // Making it so, that the alarm will trigger an interrupt
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

  SLEEP_PERIOD = TimeSpan(0, timer_period, 0, 0);

  //strobe_leds(20, 50);
  strobe_leds(timer_period, 150);

  //Serial.begin(57600);

  rtc_init();
}

// the loop function runs over and over again forever
void loop() {

  if (PLEASE_SET_ALARM) {

    setAlarm();
    PLEASE_SET_ALARM = false;
    IN_ALARM = false;
    //strobe_leds(10, 25);
    delay(500);
    // The delay seems necessary to not be fucked
    // NOPE actually you cant disable interrupts when doing I2C?
  }

  if (IN_ALARM) {
    leds_on();
  } else {
    leds_off();
  }

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void strobe_leds(int count, int period_ms) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN_BLUE, HIGH);
    digitalWrite(LED_PIN_GREEN, HIGH);
    delay(period_ms);
    digitalWrite(LED_PIN_BLUE, LOW);
    digitalWrite(LED_PIN_GREEN, LOW);
    delay(period_ms);
  }
}

void toggleLed() {
  digitalWrite(LED_PIN_BLUE, state);
  digitalWrite(LED_PIN_GREEN, !state);
  state = !state;
}

void leds_on() {
  digitalWrite(LED_PIN_BLUE, HIGH);
  digitalWrite(LED_PIN_GREEN, HIGH);
}

void leds_off() {
  digitalWrite(LED_PIN_BLUE, LOW);
  digitalWrite(LED_PIN_GREEN, LOW);
}

void onAlarm() {
  IN_ALARM = true;
}

void onButton() {
  PLEASE_SET_ALARM = true;
}

void setAlarm() {
  rtc.clearAlarm(1);
  if (!rtc.setAlarm1(
        rtc.now() + SLEEP_PERIOD,
        SLEEP_UNIT  // this mode triggers the alarm when the seconds match. See Doxygen for other options
        )) {
    strobe_leds(100, 10);
  } else {
    strobe_leds(timer_period, 75);
  }
}

void rtc_init() {

  // initializing the rtc
  if (!rtc.begin()) {
    while (1) {
      strobe_leds(10, 1000);
    }
  }

  if (rtc.lostPower()) {
    // this will adjust to the date and time at compilation
    // ... not that it matters.
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //we don't need the 32K Pin, so disable it
  rtc.disable32K();

  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  // stop oscillating signals at SQW Pin
  // otherwise setAlarm1 will fail
  rtc.writeSqwPinMode(DS3231_OFF);

  // turn off alarm 2 (in case it isn't off already)
  // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
  rtc.disableAlarm(2);

  //setAlarm(3);
  PLEASE_SET_ALARM = false;
}