// Copyright (c) 2011 Shaun Crampton.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdlib.h>
#include <stdarg.h>
#include "WProgram.h"
#include "cppboilerplate.h"
#include <avr/interrupt.h>
#include <util/atomic.h>

const int led_pin = 7;
const int pwm_a = 3;  //PWM control for motor outputs 1 and 2 is on digital pin 3
const int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
const int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12
const int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13
const int x_pin = A0;
const int y_pin = A1;
const int gyro_pin = A2;

const float GYRO_OFFSET = 4.79;
const float Y_OFFSET = 8;    // More negative tilts forwards

//#define CALIBRATION

#define RESET_TIMER1() TCNT1 = 0xC000

#define NZEROS 2
#define NPOLES 2
#define GAIN   1.013464636e+03

// Allowances for mechanical differences in motors
#define MOTOR_A_FACTOR 1
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

static float filtergyro(float in)
{
  static float xv[NZEROS+1], yv[NPOLES+1];
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / 1.565078650e+00;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
               + ( -0.4128015981 * yv[0]) + ( -1.1429805025 * yv[1]);
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

// Values read from the trim pots.
static int d_tilt_pot = 512;
static int tilt_pot = 512;
static int gyro_offset_pot = 512;

#define D_TILT_FACT ((3.5 / 512.0) * d_tilt_pot)
#define TILT_FACT ((0.025 / 512.0) * tilt_pot)
#define TILT_INT_FACT ((0.002 / 512.0) * 512.0)

static int gyro_reading = 0;
static int x_reading = 0;
static int y_reading = 0;

ISR(TIMER1_OVF_vect)
{
  float y;
  float x;
  float d_tilt;
  float speed = 0;
  static float last_speed = 0;
  long int motor_a_speed = 0;
  long int motor_b_speed = 0;
  static float tilt = 0;
  static float y_filt = 0;
  static float tilt_int = 0;
  static boolean reset_complete = false;
  static long int loop_count = 0;

  // Read the gyro rate.
  RESET_TIMER1();
  gyro_reading = analogRead(gyro_pin);
  d_tilt = (512 - gyro_reading + ((gyro_offset_pot - 512) * 0.1));

  // Read the accelerometer
  x_reading = analogRead(x_pin);
  y_reading = analogRead(y_pin);
  y = x_reading - 512 + Y_OFFSET;
  x = y_reading - 512;

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
      tilt_int = 0;
      reset_complete = true;
    }
  }
  else
  {
    // Normal operation.  Integrate gyro to get tilt.  Add in the filtered
    // accelerometer value which approximates the absolute tilt, this gives
    // us an integral term to correct for drift in the gyro.
    tilt += d_tilt + y_filt * 0.6;
    tilt_int += tilt;

#define MAX_TILT_INT (300.0 / TILT_INT_FACT)

    if (tilt_int > MAX_TILT_INT)
    {
      tilt_int = MAX_TILT_INT;
    }
    if (tilt_int < -MAX_TILT_INT)
    {
      tilt_int = -MAX_TILT_INT;
    }
#ifndef CALIBRATION
    speed = tilt * TILT_FACT +
            tilt_int * TILT_INT_FACT +
            d_tilt * D_TILT_FACT;
#endif
    last_speed = speed;
  }

  digitalWrite(dir_a, speed < 0 ? LOW : HIGH);  //Set motor direction, 1 low, 2 high
  digitalWrite(dir_b, speed < 0 ? LOW : HIGH);  //Set motor direction, 3 high, 4 low

  speed = 7 * sqrt(abs(speed));
  if (speed > 0xff)
  {
    speed = 0xff;
  }

  motor_a_speed = (long int)(speed * MOTOR_A_FACTOR);
  motor_b_speed = (long int)(speed * MOTOR_B_FACTOR);

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

  if (loop_count == 500)
  {
    d_tilt_pot = analogRead(A3);
  }
  else if (loop_count == 1000)
  {
    tilt_pot = analogRead(A5);
  }
  else if (loop_count == 1500)
  {
    gyro_offset_pot = analogRead(A4);
    loop_count = 0;
  }
  loop_count++;
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

  // Timer1 used for accurate sampling time.
  TCCR1B = _BV(CS10);   // Enable Timer 1; no prescaler
  RESET_TIMER1();
  TIMSK1 = _BV(TOIE1); // Enable Timer 1 overflow interrupt.
  sei();
}
void loop()
{
#ifdef CALIBRATION
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    myprintf("Gyro: %d, X: %d, Y: %d\r", gyro_reading, x_reading, y_reading);
    myprintf("A3: %d, A4: %d, A5: %d\r", analogRead(A3), analogRead(A4), analogRead(A5));
  }
  delay(500);
#endif
}
