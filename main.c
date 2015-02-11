// Copyright (c) 2015 Shaun Crampton.

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

#include "fixp.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h> // Make sure we pick up the optimized AVR math lib.
#include <stdint.h>
#include <stdfix.h>
#include <avr/interrupt.h>
#include "pins.h"
#include "adc.h"

#define TARGET_LOOP_HZ (500)
#define TICKS_PER_LOOP (F_CPU / TARGET_LOOP_HZ)
#define TICK_SECONDS (ACC_LITERAL(1.0) / TARGET_LOOP_HZ)

// Calculate the scaling factor from gyro ADC reading to radians.  The gyro is
// ratiometric with Vs but it doesn't use the full range.
#define RADIANS_PER_DEGREE 0.0174532925
#define V_PER_ADC_UNIT (FRACT_LITERAL(0.0048828125l))
#define GYRO_MAX_DEG_PER_SEC (ACC_LITERAL(150.0))      // From gyro datasheet
#define GYRO_V_PER_DEG_PER_SEC (ACC_LITERAL(0.01125))  // Calibrated value from datasheet.
#define GYRO_DEG_PER_ADC_UNIT (ACC_LITERAL(0.434027777)) // V_PER_ADC_UNIT / GYRO_V_PER_DEG_PER_SEC
#define GYRO_RAD_PER_ADC_UNIT (ACC_LITERAL(0.007575213759)) // (GYRO_DEG_PER_ADC_UNIT * RADIANS_PER_DEGREE)

// Calculate the scaling factor from ADC readings to g.
//
// The accelerometer is ratiometric with Vs but it doesn't use the full range
// from GND to Vs.  At Vs = 5V, it reads about 1V per g.
#define ACCEL_V_PER_G (ACC_LITERAL(1.0))
#define ACCEL_G_PER_ADC_UNIT (V_PER_ADC_UNIT / ACCEL_V_PER_G)

// EEPROM addresses
#define GYRO_OFFSET_ADDR 0
#define GYRO_OFFSET_SIZE sizeof(accum sat)
#define ACCEL_FACT_ADDR (GYRO_OFFSET_ADDR + GYRO_OFFSET_SIZE)
#define ACCEL_FACT_SIZE sizeof(accum sat)

// Default gyro offset used if no calibration has been done.  Unique to each
// gyro.
#define GYRO_OFFSET (ACC_LITERAL(3.982) * GYRO_RAD_PER_ADC_UNIT)

// Allowances for mechanical differences in motors
#define MOTOR_A_FACTOR 1
#define MOTOR_B_FACTOR 1

// Values read from the trim pots.
static uint16_t d_tilt_pot = 512;
static uint16_t x_offset_pot = 512;
static uint16_t gyro_offset_pot = 512;

static accum sat gyro_offset = GYRO_OFFSET;
static accum sat accel_fact = ACC_LITERAL(1);
static bool reset_complete = false;

static accum sat tilt_rads_estimate = ACC_LITERAL(0);
static accum sat tilt_int_rads = ACC_LITERAL(0);

// High water mark for timer values at the end of our ISR.
static long int max_timer = 0;

static accum sat speed = ACC_LITERAL(0);

#define NUM_G_SQ 3
static accum sat total_gs_sq[NUM_G_SQ];
static int gs_idx = 0;

// Counter, used to schedule work that we only do once in a while.
static long int loop_count = 0;
// true if an error has occurred like a timer overflow.
static bool error = false;

#define ABS(x) ((x) < ACC_LITERAL(0) ? -(x) : (x))

// Filter orders.
#define NZEROS 2
#define NPOLES 2

/**
 * Low pass filter. Created via
 * http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
 */
static accum sat filterx(accum sat in)
{
  static accum sat xv[NZEROS+1], yv[NPOLES+1];
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / ACC_LITERAL(1.058546241e+03);
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] =   (xv[0] + xv[2]) + ACC_LITERAL(2) * xv[1]
                        + ( -ACC_LITERAL(0.9149758348) * yv[0]) + (  ACC_LITERAL(1.9111970674) * yv[1]);
  return yv[2];
}

//static accum sat filtery(accum sat in)
//{
//  static accum sat xv[NZEROS+1], yv[NPOLES+1];
//  xv[0] = xv[1]; xv[1] = xv[2];
//  xv[2] = in / 1.058546241e+03k;
//  yv[0] = yv[1]; yv[1] = yv[2];
//  yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
//                        + ( -0.9149758348k * yv[0]) + (  1.9111970674k * yv[1]);
//  return yv[2];
//}

/**
 * (Timer 1 == OCR1A) interrupt handler.  Called each time timer 1 hits the top
 * value we set in OCR1A at start of day.
 *
 * The timer is automatically reset to 0 after it hits OCR1A.
 */
ISR(TIMER1_COMPA_vect)
{
  // First step: read the input values from the ADC.  Each ADC conversion takes
  // 0.12msec and we have a budget of 1msec per loop.  First the gyro...
  START_ADC(GYRO_APIN);
  uint16_t raw_gyro_value = wait_for_adc_result();

  // Then the X pin.
  START_ADC(X_APIN);

  // While we're waiting for the X value, convert the gyro value to rads.
  accum sat gyro_rads_per_sec = GYRO_RAD_PER_ADC_UNIT *
                                (ACC_LITERAL(512) - (accum sat)raw_gyro_value);
  gyro_rads_per_sec -= gyro_offset;

  uint16_t raw_x_value = wait_for_adc_result();

  // Then the Y pin.
  START_ADC(Y_APIN);

  // While we're waiting, convert the X value...
  accum sat x_gs = ((accum sat)ACCEL_G_PER_ADC_UNIT) * (raw_x_value - ACC_LITERAL(512));

  uint16_t raw_y_value = wait_for_adc_result();
  accum sat y_gs = ((accum sat)ACCEL_G_PER_ADC_UNIT) * (raw_y_value - ACC_LITERAL(512));

  // Start the ADC in the background while we do the rest of the processing.
  if (loop_count == TARGET_LOOP_HZ / 3) {
    START_ADC(TILT_POT_APIN);
  } else if (loop_count == (TARGET_LOOP_HZ * 2 / 3)) {
    START_ADC(X_POT_APIN);
  } else if (loop_count == TARGET_LOOP_HZ) {
    START_ADC(GYRO_POT_APIN);
  }

  // Filter the inputs from the accelerometer to try to reduce the impact of
  // vibration and acceleration from any movements the bot is making.
  accum sat x_filt_gs = filterx(x_gs) * accel_fact;
  //accum sat y_filt_gs = filtery(y_gs) * accel_fact;

  // Our constant rotation to apply to the accelerometer.  Sets the 0-tilt
  // angle.
  accum sat accel_rotation = ((accum sat)(x_offset_pot - 512) * ACC_LITERAL(0.001));

  // Estimate the tilt from the accelerometer alone.  We'll factor in a fraction
  // of this below.
  accum sat accel_tilt_estimate = x_filt_gs + accel_rotation;

  accum sat gs_sq = (x_gs * x_gs + y_gs * y_gs);
  total_gs_sq[gs_idx++] = ABS(ACC_LITERAL(1.0) - gs_sq * accel_fact * accel_fact);
  if (gs_idx == NUM_G_SQ)
  {
    gs_idx = 0;
  }

  if ((y_gs < ACC_LITERAL(0.1)) && ((x_filt_gs > -ACC_LITERAL(0.6)) || (x_filt_gs < ACC_LITERAL(0.6)))) {
    // We fell over! Shut off the motors.
    speed = 0;
    reset_complete = false;
  } else if (!reset_complete) {
    // We've never been upright, wait until we're righted by the user.
    if (-ACC_LITERAL(0.02) < x_gs && x_gs < ACC_LITERAL(0.02)) {
      tilt_rads_estimate = accel_tilt_estimate;
      speed = 0;
      tilt_int_rads = 0;
      reset_complete = true;
    }
  } else {
    // Normal operation!
#define g_factor (ACC_LITERAL(0.0005))
    tilt_rads_estimate = (ACC_LITERAL(1.0) - g_factor)
        * (tilt_rads_estimate + gyro_rads_per_sec * (accum sat)TICK_SECONDS)
        + g_factor * accel_tilt_estimate;
    tilt_int_rads += tilt_rads_estimate * (accum sat)TICK_SECONDS;

#define D_TILT_FACT ACC_LITERAL(1.0)
#define TILT_FACT ACC_LITERAL(200.0)
#define TILT_INT_FACT ACC_LITERAL(500.0)

#define MAX_TILT_INT (0.015)

    if (tilt_int_rads > (accum sat)MAX_TILT_INT) {
      tilt_int_rads = (accum sat)MAX_TILT_INT;
    }
    if (tilt_int_rads < -(accum sat)MAX_TILT_INT) {
      tilt_int_rads = -(accum sat)MAX_TILT_INT;
    }

    speed = tilt_rads_estimate * TILT_FACT +
            tilt_int_rads * TILT_INT_FACT +
            gyro_rads_per_sec * D_TILT_FACT;

    if (TCNT1 > max_timer) {
      max_timer = TCNT1;
    }
  }

  // Set the motor directions
  WRITE_PIN(DIR_A_PORT, DIR_A_PIN, speed < 0 ? PIN_OFF : PIN_ON);
  WRITE_PIN(DIR_B_PORT, DIR_B_PIN, speed < 0 ? PIN_OFF : PIN_ON);

  // Set the motor speeds.
  accum sat abs_speed = ACC_LITERAL(7) * (speed < ACC_LITERAL(0) ? -speed : speed);
  if (abs_speed > 0xff) {
    abs_speed = 0xff;
  }

  uint8_t motor_a_speed = (uint8_t)abs_speed;
  uint8_t motor_b_speed = (uint8_t)abs_speed;

  PWM_A = motor_a_speed;
  PWM_B = motor_b_speed;

  if (loop_count == TARGET_LOOP_HZ / 3) {
    d_tilt_pot = wait_for_adc_result();
  } else if (loop_count == (TARGET_LOOP_HZ * 2 / 3)) {
    x_offset_pot = wait_for_adc_result();
  } else if (loop_count == TARGET_LOOP_HZ) {
    gyro_offset_pot = wait_for_adc_result();
    loop_count = 0;
    if (error) {
      // There's been an error, turn on our status LED permanently.
      WRITE_PIN(LED_PORT, LED_PIN, PIN_ON);
    } else {
      // No error, toggle the status LED once per second as a heartbeat.
      TOGGLE_PIN(LED_PORT, LED_PIN, PIN_ON);
    }
  }
  if (bit_is_set(TIFR1, OCF1A))
  {
    // This loop took too long and the timer wrapped.  That means that our
    // timing will be incorrect.
    error = true;
  }
  loop_count++;
}

void setup() {
  // Set control pins to be outputs.
  SET_DIRECTION(PWM_A_PORT, PWM_A_PIN, DIR_OUT);
  SET_DIRECTION(PWM_B_PORT, PWM_B_PIN, DIR_OUT);
  SET_DIRECTION(DIR_A_PORT, DIR_A_PIN, DIR_OUT);
  SET_DIRECTION(DIR_B_PORT, DIR_B_PIN, DIR_OUT);

  // Turn on internal pull-up on switch input.  Note: turning on the pullup
  // uses the PORTxn register, just like writing to an output pin.
  SET_DIRECTION(SWITCH_PORT, SWITCH_PIN, DIR_IN);
  WRITE_PIN(SWITCH_PORT, SWITCH_PIN, PIN_ON);

  // Configure Timer2 for motor PWM outputs.
  // Enable PWM on each of the outputs.  Set phase-correct mode.
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS21) | _BV(CS20); // CLK/32 prescaler
  // Turn off motors for now.
  PWM_A = 0;
  PWM_B = 0;

  // Configure the Serial port.  BlueSMIRF defaults to 115200bps.
  // TODO Configure serial

  // Read calibration data from EEPROM.
  // TODO EEPROM

  // We use Timer 1 as our accurate timebase.
  // Set it to CTC mode so that we can configure its TOP value in OCR1A.
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TIMSK1 = _BV(OCIE1A); // Enable (Timer 1 == OCR1A) interrupt.
  // Have to set TOP value *after* setting up the timer or it is lost.
#if TICKS_PER_LOOP > UINT16_MAX
#error TICKS_PER_LOOP too big
#endif
  OCR1A = TICKS_PER_LOOP;

  // Configure ADC.
  // Use AVcc for reference.
  ADMUX |= _BV(REFS0);
  // Enable ADC; set CLK/128 prescaler.
  ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  // Disable digital input on the analog pins.
  DIDR0 |= _BV(ADC5D) | _BV(ADC4D) | _BV(ADC3D) |
           _BV(ADC2D) | _BV(ADC1D) | _BV(ADC0D);

  // Enable interrupts
  sei();
}

void loop() {
  __asm__("NOP");
}

int main() {
  setup();

  while (1) {
    loop();
  }
  return 0;
}
