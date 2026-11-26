/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TINYSTEP_H
#define TINYSTEP_H

#include <stdint.h>
#include <stdbool.h>

#include "tinystep_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/******* CONSTANTS ******/
/* Maximum acceleration in steps/s^2. */
#define TINYSTEP_ACCEL_488     0
#define TINYSTEP_ACCEL_977     1
#define TINYSTEP_ACCEL_1953    2
#define TINYSTEP_ACCEL_4883    3
#define TINYSTEP_ACCEL_9766    4
#define TINYSTEP_ACCEL_20020   5
#define TINYSTEP_ACCEL_49805   6
#define TINYSTEP_ACCEL_100098  7

/* Result code. */
#define TINYSTEP_RESULT_OK            0
#define TINYSTEP_RESULT_BAD_ARG       1
#define TINYSTEP_RESULT_PLATFORM_ERR  2
#define TINYSTEP_RESULT_ENCODER_FAULT 3
#define TINYSTEP_RESULT_TIMEOUT       4

/* The status of an ongoing move. */
#define TINYSTEP_STATUS_BUSY          0
#define TINYSTEP_STATUS_DONE          1
#define TINYSTEP_STATUS_ENCODER_FAULT 2
#define TINYSTEP_STATUS_PLATFORM_ERR  3

/* The minimum and maximum supported positional commands. */
#define TINYSTEP_MAX_POS  4095
#define TINYSTEP_MIN_POS (-4096)

/******** TYPEDEFS *******/

/* One of the configurable preset values of the maximum acceleration of the chip. See TINYSTEP_ACCEL_*. */
typedef uint32_t tinystep_accel;

/* The result code for a tinystep operation. See TINYSTEP_RESULT_*. */
typedef uint32_t tinystep_result;

/* The status of an ongoing move. See TINYSTEP_STATUS_*. */
typedef uint32_t tinystep_status;

/* Main driver struct, should be treated as opaque. Do not read or write these
   fields. Do not move or copy it once configured. */
typedef struct tinystep_s {
  tinystep_platform_spi *spi;
  tinystep_platform_isr_init *isr_init;
  tinystep_platform_gpio *motor_enable;
  tinystep_platform_event *event;
  tinystep_accel max_accel;
  bool is_configured;
} tinystep;

/******** FUNCTIONS ********/

/**
 * Initialize the tinystep driver. This is the only supported way to construct a `tinystep` object.
 *
 * @param[in] spi Platform-specific SPI context.
 * @param[in] isr_init Platform-specific interrupt registration context.
 * @param[in] enable Platform-specific context for the chip's motion enable input.
 * @param[in] event Platform-specific post/wait event used by the interrupt path.
 * @param[in] max_accel One of the TINYSTEP_ACCEL_* presets; applied by tinystep_config().
 * @returns The initialized tinystep driver, in the unconfigured state.
 */
tinystep tinystep_init(
  tinystep_platform_spi *spi,
  tinystep_platform_isr_init *isr_init,
  tinystep_platform_gpio *enable,
  tinystep_platform_event *event,
  tinystep_accel max_accel
);

/**
 * Configure the driver, writing to the SPI bus and setting up interrupts.
 *
 * @param[in] ts The driver.
 * @returns Zero on a success, a negative integer for a platform-level non-retryable error, 
 *          a positive number for a retryable error that exceeded the retry budget.
 */
int tinystep_config(tinystep *ts);

/**
 * Enable the motor output of the driver.
 * 
 * @param[in] ts The driver, already configured by tinystep_config().
 * @returns Zero on success, the platform-specific error code on failure.
 */
int tinystep_enable(tinystep *ts);

/**
 * Disable the motor output of the driver.
 * 
 * @param[in] ts The driver.
 * @returns Zero on success, the platform-specific error code on failure.
 */
int tinystep_disable(tinystep *ts);

/**
 * Start moving towards the specified position.
 *
 * @param[in] ts The driver, already configured by tinystep_config().
 * @param[in] pos The target position measured in steps/encoder ticks (which should be 1:1).
 *                Must be within TINYSTEP_MIN_POS..TINYSTEP_MAX_POS (-4096..4095).
 * @param[out] platform_error The platform-specific error code from the last attempt,
 *                            or zero on success.
 * @returns TINYSTEP_RESULT_OK if the command was accepted, TINYSTEP_RESULT_BAD_ARG
 *          if `pos` is outside the supported range, TINYSTEP_RESULT_PLATFORM_ERR on
 *          a non-retryable platform error.
 */
tinystep_result tinystep_async_move_start(
  tinystep *ts,
  int32_t pos,
  int *platform_error
);

/**
 * Check the status of an ongoing move command.
 *
 * @param[in] ts The driver, already configured by tinystep_config().
 * @param[out] platform_error The platform-specific error code from the last attempt,
 *                            or zero on success.
 * @returns The status of the move. A status of TINYSTEP_STATUS_BUSY indicates that the
 *          move command is ongoing, and all other statuses indicate it has completed.
 */
tinystep_status tinystep_async_move_status(
  tinystep *ts,
  int *platform_error
);

/**
 * Start moving towards the specified position, then block until it has completed, or a stall occurs.
 *
 * @param[in] ts The driver, already configured by tinystep_config().
 * @param[in] pos The target position measured in steps/encoder ticks (which should be 1:1).
 *                Must be within TINYSTEP_MIN_POS..TINYSTEP_MAX_POS (-4096..4095).
 * @param[in] max_polls The maximum number of status polls before the move is reported
 *                      as TINYSTEP_RESULT_TIMEOUT. Must not exceed INT32_MAX.
 * @param[out] platform_error The platform-specific error code when the result is
 *                            TINYSTEP_RESULT_PLATFORM_ERR, otherwise zero.
 * @returns TINYSTEP_RESULT_OK when the move settled on the target,
 *          TINYSTEP_RESULT_BAD_ARG if `pos` is outside the supported range,
 *          TINYSTEP_RESULT_PLATFORM_ERR on a non-retryable platform error,
 *          TINYSTEP_RESULT_ENCODER_FAULT if the chip reported an encoder fault,
 *          TINYSTEP_RESULT_TIMEOUT if the move did not settle within `max_polls` polls.
 */
tinystep_result tinystep_move(
  tinystep *ts,
  int32_t pos,
  uint32_t max_polls,
  int *platform_error
);

#ifdef __cplusplus
}
#endif

#endif /* TINYSTEP_H */
