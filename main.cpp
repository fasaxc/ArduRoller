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

#define TARGET_LOOP_HZ (1000)
#define TICKS_PER_LOOP (F_CPU / TARGET_LOOP_HZ)
#define TICK_SECONDS (TICKS_PER_LOOP / F_CPU)

// Arduino has a 10-bit ADC, giving a range of 0-1023.  0 represents GND, 1023
// represents Vcc - 1 LSB.  Hence, 512 is the midpoint.
#define ADC_RANGE (1024)

// Calculate the scaling factor from gyro ADC reading to radians.
#define GYRO_MAX_DEG_PER_SEC (150.0)
#define GYRO_DEG_PER_ADC_UNIT (GYRO_MAX_DEG_PER_SEC * 2 / ADC_RANGE)
#define GYRO_RAD_PER_ADC_UNIT (GYRO_DEG_PER_ADC_UNIT * 0.0174532925)

// Calculate the scaling factor from ADC readings to g.  We use the g reading
// from X-axis sensor as an approximation for the tilt angle (since, for small
// angles sin(x) (which the accelerometer measures) is approximately equal to x.
#define ACCEL_MAX_G (1.7)
#define ACCEL_G_PER_ADC_UNIT (ACCEL_MAX_G * 2 / ADC_RANGE)

// Pin definitions
const int pwm_a = 3;   // PWM control for motor outputs 1 and 2 is on digital pin 3
const int pwm_b = 11;  // PWM control for motor outputs 3 and 4 is on digital pin 11
const int dir_a = 12;  // direction control for motor outputs 1 and 2 is on digital pin 12
const int dir_b = 13;  // direction control for motor outputs 3 and 4 is on digital pin 13
const int x_pin = A0;
const int y_pin = A1;
const int gyro_pin = A2;

const float GYRO_OFFSET = 4.79;
const float X_OFFSET = 8;    // More negative tilts forwards

//#define CALIBRATION

#define NZEROS 2
#define NPOLES 2
#define GAIN   1.013464636e+03

// Allowances for mechanical differences in motors
#define MOTOR_A_FACTOR 1
#define MOTOR_B_FACTOR 1

static float filterx(float in)
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

#define D_TILT_FACT ((3.5 / 512.0) * d_tilt_pot / GYRO_RAD_PER_ADC_UNIT)
#define TILT_FACT ((0.025 / 512.0) * tilt_pot / GYRO_RAD_PER_ADC_UNIT)
#define TILT_INT_FACT ((0.002 / 512.0) * 512.0 / GYRO_RAD_PER_ADC_UNIT)

static int gyro_reading = 0;
static int x_reading = 0;
static int y_reading = 0;

static long int max_timer = 0;

/**
 * (Timer 1 == OCR1A) interrupt handler.  Called each time timer 1 hits the top
 * value we set in OCR1A at start of day.
 *
 * The timer is automatically reset to 0 after it hits OCR1A.
 */
ISR(TIMER1_COMPA_vect)
{
  float y_gs;
  float x_gs;
  float gyro_rads_per_sec;
  float speed = 0;
  static float last_speed = 0;
  long int motor_a_speed = 0;
  long int motor_b_speed = 0;
  static float tilt_rads = 0;
  static float x_filt_gs = 0;
  static float tilt_int_rads = 0;
  static boolean reset_complete = false;
  static long int loop_count = 0;

  // Read the inputs.  Each analog read should take about 0.12 msec.  We can't
  // do too many analog reads per timer tick.

  // Gyro rate.
  gyro_reading = analogRead(gyro_pin);
  // Accelerometer
  x_reading = analogRead(x_pin);
  y_reading = analogRead(y_pin);

  // Convert to sensible units
  float gyro_offset = ((gyro_offset_pot - 512) * 0.1);
  gyro_rads_per_sec =  GYRO_RAD_PER_ADC_UNIT * (512 - gyro_reading + gyro_offset);
  x_gs = ACCEL_G_PER_ADC_UNIT * (x_reading - 512 + X_OFFSET);
  y_gs = ACCEL_G_PER_ADC_UNIT * (y_reading - 512);

  x_filt_gs = filterx(x_gs);

  if (y_gs < 0.1 && abs(x_filt_gs) > 0.6)
  {
    // We fell over! Shut off the motors.
    speed = 0;
    reset_complete = false;
  }
  else if (!reset_complete)
  {
    // We've never been upright, wait until we're righted by the user.
    if (-0.02 < x_gs && x_gs < 0.02)
    {
      tilt_rads = x_gs;
      tilt_int_rads = 0;
      reset_complete = true;
    }
  }
  else
  {
    // Normal operation.  Integrate gyro to get tilt.  Add in the filtered
    // accelerometer value which approximates the absolute tilt, this gives
    // us an integral term to correct for drift in the gyro.
    tilt_rads += gyro_rads_per_sec * TICK_SECONDS + x_filt_gs;
    tilt_int_rads += tilt_rads;

#define MAX_TILT_INT (300.0 * GYRO_RAD_PER_ADC_UNIT / TILT_INT_FACT)

    if (tilt_int_rads > MAX_TILT_INT)
    {
      tilt_int_rads = MAX_TILT_INT;
    }
    if (tilt_int_rads < -MAX_TILT_INT)
    {
      tilt_int_rads = -MAX_TILT_INT;
    }
#ifndef CALIBRATION
    speed = tilt_rads * TILT_FACT +
            tilt_int_rads * TILT_INT_FACT +
            gyro_rads_per_sec * D_TILT_FACT;
#endif
    last_speed = speed;

    if (TCNT1 > max_timer)
    {
      max_timer = TCNT1;
    }
  }

  // Set the motor directions
  digitalWrite(dir_a, speed < 0 ? LOW : HIGH);
  digitalWrite(dir_b, speed < 0 ? LOW : HIGH);

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

  // We use Timer 1 as our accurate timebase.
  // Set it to CTC mode so that we can configure its TOP value in OCR1A.
  OCR1A = TICKS_PER_LOOP;
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TIMSK1 = _BV(OCIE1A);           // Enable (Timer 1 == OCR1A) interrupt.

  // Enable interrupts
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
