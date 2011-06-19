#include <stdlib.h>
#include <stdarg.h>
#include "WProgram.h"
#include "cppboilerplate.h"
#include <avr/interrupt.h>

int pwm_a = 3;  //PWM control for motor outputs 1 and 2 is on digital pin 3
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13
int x_pin = A0;
int y_pin = A1;
int gyro_pin = A2;

const float GYRO_OFFSET = 3.45;
const float Y_OFFSET = -3.7;    // More negative tilts forwards

#define RESET_TIMER1() TCNT1 = 0xC000

void myprintf(const char *fmt, ... ){
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  Serial.print(tmp);
}

ISR(TIMER1_OVF_vect)
{
  float y;
  float x;
  float d_tilt;
  long int speed = 0;
  static float tilt = 0;
  static float y_filt = 0;
  static boolean reset_complete = false;

  // Read the gyro rate.
  d_tilt = 512 - analogRead(gyro_pin) + GYRO_OFFSET;
  RESET_TIMER1();

  // Read the accelerometer
  y = analogRead(x_pin) - 512 + Y_OFFSET;
  x = analogRead(y_pin) - 512;

  y_filt = y_filt * 0.87 + y * 0.12;

  if (x < 30)
  {
    // On our front/back. Shut off the motors.
    speed = 0;
  }
  else if (!reset_complete)
  {
    if (-5 < y && y < 5)
    {
      tilt = 0;
      reset_complete = true;
    }
  }
  else
  {
    // Normal operation
    tilt += d_tilt + y_filt / 2.0;
    speed = tilt * 0.003 + d_tilt * 0.20;
  }

  digitalWrite(dir_a, speed < 0 ? LOW : HIGH);  //Set motor direction, 1 low, 2 high
  digitalWrite(dir_b, speed < 0 ? LOW : HIGH);  //Set motor direction, 3 high, 4 low

  speed = 16 * sqrt((float)abs(speed));

  if (speed > 0xff)
  {
    speed = 0xff;
  }

  analogWrite(pwm_a, speed);
  analogWrite(pwm_b, speed);
}

void setup()
{
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  analogWrite(pwm_a, 0);
  analogWrite(pwm_b, 0);

  Serial.begin(9600);
  Serial.println("Starting up");

  // Timer0 PWM
  TCCR0B = (TCCR0B & 0xf8) | _BV(CS02);
  TCCR2B = (TCCR0B & 0xf8) | _BV(CS02);

  // Timer1 configuration
  TCCR1B = _BV(CS10);   // Enable Timer 1; no prescaler
  RESET_TIMER1();
  TIMSK1 = _BV(TOIE1); // Enable Timer 1 overflow interrupt.
  sei();
}

void loop()
{
  // Nothing to do... the ISR does it all
}
