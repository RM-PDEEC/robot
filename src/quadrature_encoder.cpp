/**
 * Copyright (c) 2021 pmarques-dev @ github
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"

#include "quadrature_encoder.pio.h"

//
// ---- quadrature encoder interface example
//
// the PIO program reads phase A/B of a quadrature encoder and increments or
// decrements an internal counter to keep the current absolute step count
// updated. At any point, the main code can query the current count by using
// the quadrature_encoder_*_count functions. The counter is kept in a full
// 32 bit register that just wraps around. Two's complement arithmetic means
// that it can be interpreted as a 32-bit signed or unsigned value, and it will
// work anyway.
//
// As an example, a two wheel robot being controlled at 100Hz, can use two
// state machines to read the two encoders and in the main control loop it can
// simply ask for the current encoder counts to get the absolute step count. 
// One advantage of this approach is that it requires zero CPU time to keep the
// encoder count updated and because of that it supports very high step rates.
//


// encx_pin_A -> Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
void init_PIO_dual_encoders(int enc1_pin_A, int enc2_pin_A) 
{
  // Mark all state machines from pio0 as unavailable
  for (int i = 0; i < 4; i++) {
    pio_sm_claim(pio0, i);
  }
  uint offset = pio_add_program(pio0, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio0, 0, offset, enc1_pin_A, 0);
  quadrature_encoder_program_init(pio0, 1, offset, enc2_pin_A, 0);
}

// encx_pin_A pointer to an array with the pin number for the A phase of each encoder
// The B phase must be connected to the next pin
// num_encoders must be between 1 and 4
int init_PIO_encoders(int enc_pins_A[], int num_encoders) 
{
  if (num_encoders < 1 || num_encoders > 4) return -1; 
  
  // Mark all state machines from pio0 as unavailable
  for (int i = 0; i < 4; i++) {
    pio_sm_claim(pio0, i);
  }

  uint offset = pio_add_program(pio0, &quadrature_encoder_program);
  for (int i = 0; i < num_encoders; i++) {
    quadrature_encoder_program_init(pio0, i, offset, enc_pins_A[i], 0);
  }
  return offset;
}



int read_PIO_encoder(int sm)
{
  return quadrature_encoder_get_count(pio0, sm);
}

