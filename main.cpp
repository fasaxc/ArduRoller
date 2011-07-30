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
#define TICK_SECONDS ((float)TICKS_PER_LOOP / F_CPU)

// Arduino has a 10-bit ADC, giving a range of 0-1023.  0 represents GND, 1023
// represents Vcc - 1 LSB.  Hence, 512 is the midpoint.
#define ADC_RANGE (1024)
#define ADC_HALF_RANGE (512)

// Calculate the scaling factor from gyro ADC reading to radians.
#define V_PER_ADC_UNIT (5.0 / ADC_RANGE)
#define GYRO_MAX_DEG_PER_SEC (150.0)
#define GYRO_V_PER_DEG_PER_SEC (12.5 / 1000.0)
// Swing at 5V
#define GYRO_DEG_PER_ADC_UNIT (V_PER_ADC_UNIT / GYRO_V_PER_DEG_PER_SEC)
#define GYRO_RAD_PER_ADC_UNIT (GYRO_DEG_PER_ADC_UNIT * 0.0174532925)

// Calculate the scaling factor from ADC readings to g.  We use the g reading
// from X-axis sensor as an approximation for the tilt angle (since, for small
// angles sin(x) (which the accelerometer measures) is approximately equal to x.
//
// The accelerometer is ratiometric with Vs but it doesn't use the full range
// from GND to Vs.  At 5V, it reads about 1V per g.
#define ACCEL_VS_PER_G (1.080 / 5.0)
#define ACCEL_G_PER_ADC_UNIT (1.0 / (ADC_RANGE * ACCEL_VS_PER_G))

// Pin definitions
const int led_pin = 7;
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

// Allowances for mechanical differences in motors
#define MOTOR_A_FACTOR 1
#define MOTOR_B_FACTOR 1

#define NZEROS 2
#define NPOLES 2

/**
 * Low pass filter. Created via
 * http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
 */
static float filterx(float in)
{
  static float xv[NZEROS+1], yv[NPOLES+1];
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / 1.058546241e+03;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] = (xv[0] + xv[2]) + 2 * xv[1]
                                   + ( -0.9149758348 * yv[0]) + (  1.9111970674 * yv[1]);
  return yv[2];
}

/**
 *
 */
static float filtergyro(float in)
{
  static float xv[NZEROS+1], yv[NPOLES+1];
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / 1.000004443e+00;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] = (xv[0] + xv[2]) - 2 * xv[1]
                          + ( -0.9999911143 * yv[0]) + (  1.9999911142 * yv[1]);
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

static int gyro_reading = 0;
static int x_reading = 0;
static int y_reading = 0;

static float last_speed = 0;
static float tilt_rads_estimate = 0;
static float x_filt_gs = 0;
static float tilt_int_rads = 0;
static boolean reset_complete = false;
static long int loop_count = 0;
static uint8_t error = 0;
static long int max_timer = 0;

static bool write_gyro = false;

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
  long int motor_a_speed = 0;
  long int motor_b_speed = 0;
  static uint16_t counter = 0;

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
  float x_raw_gs = ACCEL_G_PER_ADC_UNIT * (x_reading - 512);
  y_gs = ACCEL_G_PER_ADC_UNIT * (y_reading - 512);

  x_filt_gs = filterx(x_gs);

  float total_gs_sq = x_raw_gs * x_raw_gs + y_gs * y_gs;

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
      tilt_rads_estimate = x_gs;
      tilt_int_rads = 0;
      reset_complete = true;
    }
  }
  else
  {
    // The accelerometer isn't trustworthy if it's not reporting exactly 1G of
    // force (it must be picking up acceleration from the motors).
    float g_factor = max(0, 1.0 - (100 * abs(1.0 - total_gs_sq))) * 0.05;

    tilt_rads_estimate = (1.0 - g_factor) * (tilt_rads_estimate + gyro_rads_per_sec * TICK_SECONDS) +
                         g_factor * x_gs;
    tilt_int_rads += tilt_rads_estimate;

#define D_TILT_FACT 100.0
#define TILT_FACT 20000.0
#define TILT_INT_FACT 0.0

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
    speed = tilt_rads_estimate * TILT_FACT +
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

  if (write_gyro && counter >= 100)
  {
    Serial.print('g');
    Serial.print(gyro_reading);
    Serial.print(" f");
    Serial.print(gyro_rads_per_sec, 4);
    Serial.print(" t");
    Serial.print(tilt_rads_estimate, 4);
    Serial.print(" x");
    Serial.print(x_reading);
    Serial.print(" x");
    Serial.print(x_raw_gs, 4);
    Serial.print(" y");
    Serial.print(y_gs, 4);
    Serial.print(" t");
    Serial.print(total_gs_sq, 4);
    Serial.print(" s");
    Serial.print(speed);
    Serial.print('\n');
    Serial.flush();
    counter = 0;
  }
  else
  {
    counter ++;
  }

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

  if (loop_count == TARGET_LOOP_HZ / 3)
  {
    d_tilt_pot = analogRead(A3);
  }
  else if (loop_count == (TARGET_LOOP_HZ * 2 /3))
  {
    tilt_pot = analogRead(A5);
  }
  else if (loop_count == TARGET_LOOP_HZ)
  {
    gyro_offset_pot = analogRead(A4);
    loop_count = 0;
    if (error)
    {
      digitalWrite(led_pin, HIGH);
    }
    else
    {
      digitalWrite(led_pin, !digitalRead(led_pin));
    }
  }
  if (TCNT1 > TICKS_PER_LOOP)
  {
    error = 1;
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

  Serial.begin(115200);

  // Timer0 PWM
  TCCR0B = (TCCR0B & 0xf8) | _BV(CS02);
  TCCR2B = (TCCR0B & 0xf8) | _BV(CS02);

  // We use Timer 1 as our accurate timebase.
  // Set it to CTC mode so that we can configure its TOP value in OCR1A.
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TIMSK1 = _BV(OCIE1A);           // Enable (Timer 1 == OCR1A) interrupt.
  // Have to set the TOP value after setting up the timer or it doesn't take
  // effect.
  OCR1A = TICKS_PER_LOOP;

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
  int c = Serial.read();
  {
    if (c == 'g')
    {
      write_gyro = true;
    }
  }
}
