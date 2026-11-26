/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TINYSTEP_PLATFORM_H
#define TINYSTEP_PLATFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******* TYPEDEFS ******/

struct tinystep_s;

/* Unsized struct type for platform-specific SPI context. */
typedef struct tinystep_platform_spi_s tinystep_platform_spi;

/* Unsized struct type for platform-specific ISR initialization context. */
typedef struct tinystep_platform_isr_init_s tinystep_platform_isr_init;

/* Unsized struct type for platform-specific GPIO writes. */
typedef struct tinystep_platform_gpio_s tinystep_platform_gpio;

/* Unsized struct type for platform-specific post/wait style events. */
typedef struct tinystep_platform_event_s tinystep_platform_event;

/** 
 * Function pointer type for ISR callbacks.
 *
 * @param[in] ts The tinystep driver to be passed in as context.
 */
typedef void (*tinystep_isr)(struct tinystep_s *ts);

/***** FUNCTIONS ******/

/**
 * Perform a 16 bit SPI transfer, including CS assertion/deassertion.
 * 
 * @param[in] spi Platform-specific SPI context.
 * @param[in] tx 16 bits to be sent, MSB first.
 * @param[out] rx Resulting 16 bits acquired from the transfer.May be NULL, in which case the received bits should be discarded.
 * @returns Zero on success, a negative number for a non-retryable error, a positive number for a retryable error.
 */
int tinystep_platform_xfer16(tinystep_platform_spi *spi, uint16_t tx, uint16_t *rx);

/**
 * Register and enable an ISR, to be fired whenever there is a positive edge on the INT line of the chip.
 *
 * @param[in] isr_init Platform-specific context for initializing interrupts.
 * @param[in] ts The driver state, which should be passed to the ISR when it is triggered.
 * @param[in] isr The interrupt service routine.
 * @returns Zero on success, a negative number for a non-retryable error, a positive number for a retryable error.
 */
int tinystep_platform_register_isr(tinystep_platform_isr_init *isr_init, struct tinystep_s *ts, tinystep_isr isr);

/**
 * Write to a specific GPIO block and channel.
 *
 * @param[in] gpio Platform-specific struct for which GPIO block/channel to write to.
 * @param[in] value The value to write to the GPIO.
 * @returns Zero on success, a negative number for a non-retryable error, a positive number for a retryable error.
 */
int tinystep_platform_write_gpio(tinystep_platform_gpio *gpio, bool value);

/**
 * Indicate the given event is unblocked.
 * 
 * @param[in] event The event to unblock.
 * @returns Zero on success, a negative number for a non-retryable error, a positive number for a retryable error.
 */
int tinystep_platform_post_event(tinystep_platform_event *event);

/**
 * Check if the given event is blocked.
 * 
 * @param[in] event The event to check.
 * @returns Zero if the event is unblocked, a negative number for a non-retryable error, a positive number for a retryable error, including the event being blocked.
 */
int tinystep_platform_wait_event(tinystep_platform_event *event);

#ifdef __cplusplus
}
#endif

#endif /* TINYSTEP_PLATFORM_H */
