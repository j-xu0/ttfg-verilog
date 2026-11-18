/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TINYSTEP_PLATFORM_HOST_H
#define TINYSTEP_PLATFORM_HOST_H

#include <stdbool.h>
#include <stdint.h>

#include "tinystep.h"

/*
 * Host platform: a simulated chip, for tests. Same 16-bit frame protocol as
 * src/stepper_spi.v -- writes silently rejected while busy, any read acks
 * sticky faults, INT on done or fault, motion-enable low aborts. No hardware,
 * no allocation, no threads: simulated time advances one control tick per
 * wait_event call.
 */

tinystep_platform_spi *tinystep_host_spi(void);
tinystep_platform_isr_init *tinystep_host_isr_init(void);
tinystep_platform_gpio *tinystep_host_enable(void);
tinystep_platform_event *tinystep_host_event(void);

void tinystep_host_reset(void);

/* Frame log since reset; index 0 is the first transfer. */
int tinystep_host_transfers(void);
uint16_t tinystep_host_tx_at(int index);

/* Chip state. */
bool tinystep_host_busy(void);
bool tinystep_host_at_target(void);
bool tinystep_host_fault_latched(void);
int16_t tinystep_host_target(void);
int16_t tinystep_host_position(void);
uint8_t tinystep_host_plan(void);
bool tinystep_host_enable_level(void);
int tinystep_host_ticks(void);

/* Stimulus. */
void tinystep_host_set_stall(bool stalled);
void tinystep_host_inject_fault(bool range_fault, bool encoder_fault);

#endif /* TINYSTEP_PLATFORM_HOST_H */
