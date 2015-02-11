/*
 * pins.h
 *
 *  Created on: Feb 8, 2015
 *      Author: shaun
 */

#ifndef PINS_H_
#define PINS_H_

#define SET_DIRECTION_INNER(port, pin, out) if (out) {\
  DDR##port |= _BV(DD##port##pin); \
} else { \
  DDR##port &= ~_BV(DD##port##pin); \
}
#define SET_DIRECTION(port, pin, out) SET_DIRECTION_INNER(port, pin, out)
#define DIR_OUT 1
#define DIR_IN 0

#define WRITE_PIN_INNER(port, pin, out) do { if (out) {\
  PORT##port |= _BV(PORT##port##pin); \
} else { \
  PORT##port &= ~_BV(PORT##port##pin); \
} } while (0)
#define WRITE_PIN(port, pin, out) WRITE_PIN_INNER(port, pin, out)

#define TOGGLE_PIN_INNER(port, pin, out) do { if (out) {\
  PIN##port |= _BV(PIN##port##pin); \
} else { \
  PIN##port &= ~_BV(PIN##port##pin); \
} } while (0)
#define TOGGLE_PIN(port, pin, out) TOGGLE_PIN_INNER(port, pin, out)

#define PIN_ON 1
#define PIN_OFF 0



// Pin definitions
#define LED_PORT D
#define LED_PIN 7

// PWM A = PD3 = OC2B // PWM control for motor outputs 1 and 2 is on digital pin 3
#define PWM_A_PORT D
#define PWM_A_PIN 3
#define PWM_A OCR2B
// PWM B = PB3 = OC2A // PWM control for motor outputs 3 and 4 is on digital pin 11
#define PWM_B_PORT B
#define PWM_B_PIN 3
#define PWM_B OCR2A

// direction control for motor outputs 1 and 2 is on Arduino digital pin 12
#define DIR_A_PORT B
#define DIR_A_PIN 4
// direction control for motor outputs 3 and 4 is on Arduino digital pin 13
#define DIR_B_PORT B
#define DIR_B_PIN 5

// Pin connected to our active-low push switch.
#define SWITCH_PORT B
#define SWITCH_PIN 0

// Analog pins assignments.
#define X_APIN 0
#define Y_APIN 1
#define GYRO_APIN 2
#define TILT_POT_APIN 3
#define GYRO_POT_APIN 4
#define X_POT_APIN 5


#endif /* PINS_H_ */
