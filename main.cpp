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
const float Y_OFFSET = -7;    // More negative tilts forwards

#define RESET_TIMER1() TCNT1 = 0xC000

#define NZEROS 2
#define NPOLES 2
#define GAIN   1.013464636e+03

// Allowances for mechanical differences in motors
#define MOTOR_A_FACTOR 107 / 100
#define MOTOR_B_FACTOR 1

static float filtery(float in)
{
  static float xv[NZEROS+1], yv[NPOLES+1];
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / GAIN;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
               + ( -0.9131487721 * yv[0]) + (  1.9092019151 * yv[1]);
  return yv[2];
 }

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
  long int last_speed = 0;
  long int motor_a_speed = 0;
  long int motor_b_speed = 0;
  static float tilt = 0;
  static float y_filt = 0;
  static float y_filt_int = 0;
  static float tilt_int = 0;
  static boolean reset_complete = false;

  // Read the gyro rate.
  RESET_TIMER1();
  d_tilt = 512 - analogRead(gyro_pin) + GYRO_OFFSET;

  // Read the accelerometer
  y = analogRead(x_pin) - 512 + Y_OFFSET;
  x = analogRead(y_pin) - 512;

  y_filt = filtery(y);

  if (x < 30 && abs(y_filt) > 150)
  {
    // We fell over! Shut off the motors.
    speed = 0;
    reset_complete = false;
  }
  else if (!reset_complete)
  {
    // We've never been upright, wait until we're righted by the user.
    if (-5 < y && y < 5)
    {
      tilt = y;
      reset_complete = true;
    }
  }
  else
  {
    // Normal operation.  Integrate gyro to get tilt.  Add in the filtered
    // accelerometer value which approximates the absolute tilt, this gives
    // us an integral term to correct for drift in the gyro.
    tilt += d_tilt;
    tilt_int += tilt;
    y_filt_int += y_filt;
    speed = tilt * 0.003 +
            y_filt_int * 0.003 +
            d_tilt * 0.20;
    last_speed = speed;
  }

  digitalWrite(dir_a, speed < 0 ? LOW : HIGH);  //Set motor direction, 1 low, 2 high
  digitalWrite(dir_b, speed < 0 ? LOW : HIGH);  //Set motor direction, 3 high, 4 low

  speed = 16 * sqrt((float)abs(speed));

  motor_a_speed = speed * MOTOR_A_FACTOR;
  motor_b_speed = speed * MOTOR_B_FACTOR;

  if (motor_a_speed > 0xff)
  {
    motor_a_speed = 0xff;
  }
  if (motor_b_speed > 0xff)
  {
    motor_b_speed = 0xff;
  }

  analogWrite(pwm_a, motor_a_speed);
  analogWrite(pwm_b, motor_b_speed);
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
