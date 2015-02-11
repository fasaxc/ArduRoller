/*
 * adc.h
 *
 *  Created on: Feb 10, 2015
 *      Author: shaun
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

// Arduino has a 10-bit ADC, giving a range of 0-1023.  0 represents GND, 1023
// represents Vcc - 1 LSB.  Hence, 512 is the midpoint.
#define ADC_RANGE (1024)
#define ADC_HALF_RANGE (512)


#define SELECT_ADC_CHANNEL(apin) ADMUX = (ADMUX & 0b11110000) | apin
#define START_ADC(apin) \
  SELECT_ADC_CHANNEL(apin); \
  ADCSRA |= _BV(ADSC)


uint16_t wait_for_adc_result() {
  while (bit_is_set(ADCSRA, ADSC)) {
    // Wait.
  }
  return ADCW;
}


#endif /* ADC_H_ */
