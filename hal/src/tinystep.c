/* SPDX-License-Identifier: Apache-2.0 */

#include <stddef.h>
#include <stdint.h>

#include "tinystep.h"
#include "tinystep_assert.h"

/****** PRIVATE CONSTANTS *******/
#define TINYSTEP_PLATFORM_ATTEMPTS 3

/****** PRIVATE FUNCTIONS ********/
static void tinystep_isr_callback(tinystep *ts) {
  // Explicitly don't assert in the ISR.

  if (ts != NULL && ts->is_configured)
    tinystep_platform_post_event(ts->event);
}

static int tinystep_set_enable(tinystep *ts, bool value) {
  TINYSTEP_ASSERT(ts != NULL);
  TINYSTEP_ASSERT(ts -> is_configured);

  uint32_t retry_budget = TINYSTEP_PLATFORM_ATTEMPTS;
  int result = 0;
  do {
    result = tinystep_platform_write_gpio(ts->motor_enable, value);
  } while (result > 0 && (--retry_budget) > 0);

  return result;
}

static bool tinystep_accel_is_valid(tinystep_accel max_accel) {
  return max_accel == TINYSTEP_ACCEL_488
    || max_accel == TINYSTEP_ACCEL_977
    || max_accel == TINYSTEP_ACCEL_1953
    || max_accel == TINYSTEP_ACCEL_4883
    || max_accel == TINYSTEP_ACCEL_9766
    || max_accel == TINYSTEP_ACCEL_20020
    || max_accel == TINYSTEP_ACCEL_49805
    || max_accel == TINYSTEP_ACCEL_100098;
}

/****** PUBLIC FUNCTIONS ********/

tinystep tinystep_init(
  tinystep_platform_spi *spi,
  tinystep_platform_isr_init *isr_init,
  tinystep_platform_gpio *motor_enable,
  tinystep_platform_event *event,
  tinystep_accel max_accel
) {
  TINYSTEP_ASSERT(spi != NULL);
  TINYSTEP_ASSERT(isr_init != NULL);
  TINYSTEP_ASSERT(motor_enable != NULL);
  TINYSTEP_ASSERT(event != NULL);
  TINYSTEP_ASSERT(tinystep_accel_is_valid(max_accel));

  return (tinystep) {
    .spi = spi,
    .isr_init = isr_init,
    .motor_enable = motor_enable,
    .event = event,
    .max_accel = max_accel,
    .is_configured = false
  };
}

int tinystep_config(tinystep *ts) {
  TINYSTEP_ASSERT(ts != NULL);

  if (ts->is_configured) return 0;

  // Write Plan
  // Mask, not validation: the assert above is the check; this keeps an out-of-range
  // accel from spilling into the address field and hitting another register.
  uint16_t tx = (1 << 15) | (2 << 13) | (ts->max_accel & 0x7);
  uint32_t retry_budget = TINYSTEP_PLATFORM_ATTEMPTS;
  int result = 0;
  do {
    result = tinystep_platform_xfer16(ts->spi, tx, NULL);
  } while (result > 0 && (--retry_budget) > 0);

  if (result != 0) return result;

  // Clear sticky faults
  retry_budget = TINYSTEP_PLATFORM_ATTEMPTS;
  do {
    result = tinystep_platform_xfer16(ts->spi, 0, NULL);
  } while (result > 0 && (--retry_budget) > 0);

  if (result != 0) return result;

  // Register interrupt
  retry_budget = TINYSTEP_PLATFORM_ATTEMPTS;
  do {
    result = tinystep_platform_register_isr(ts->isr_init, ts, tinystep_isr_callback);
  } while (result > 0 && (--retry_budget) > 0);

  if (result == 0) ts->is_configured = true;

  return result;
}

int tinystep_enable(tinystep *ts) {
  TINYSTEP_ASSERT(ts != NULL);
  TINYSTEP_ASSERT(ts -> is_configured);

  return tinystep_set_enable(ts, true);
}

int tinystep_disable(tinystep *ts) {
  TINYSTEP_ASSERT(ts != NULL);

  return tinystep_set_enable(ts, false);
}

tinystep_result tinystep_async_move_start(
  tinystep *ts,
  int32_t pos,
  int *platform_error
) {
  TINYSTEP_ASSERT(ts != NULL);
  TINYSTEP_ASSERT(platform_error != NULL);
  TINYSTEP_ASSERT(ts -> is_configured);
  if (pos > TINYSTEP_MAX_POS || pos < TINYSTEP_MIN_POS) return TINYSTEP_RESULT_BAD_ARG;

  uint16_t tx = 0x8000 | (pos & 0x1FFF);
  uint16_t rx = 0;
  uint32_t retry_budget = TINYSTEP_PLATFORM_ATTEMPTS;
  do {
    *platform_error = tinystep_platform_xfer16(ts->spi, tx, &rx);
    (void) rx; // insert correct error handling here, pending finalization of ASIC level error handling.
  } while (*platform_error > 0 && (--retry_budget) > 0);

  if (*platform_error == 0) return TINYSTEP_RESULT_OK;
  else return TINYSTEP_RESULT_PLATFORM_ERR;
}

tinystep_status tinystep_async_move_status(
  tinystep *ts,
  int *platform_error
) {
  TINYSTEP_ASSERT(ts != NULL);
  TINYSTEP_ASSERT(platform_error != NULL);
  TINYSTEP_ASSERT(ts -> is_configured);

  uint16_t tx = 0;
  uint16_t rx = 0;
  uint32_t retry_budget = TINYSTEP_PLATFORM_ATTEMPTS;
  do {
    *platform_error = tinystep_platform_xfer16(ts->spi, tx, &rx);
    (void) rx; // TODO: insert correct error handling here, pending finalization of ASIC level error handling.
  } while (*platform_error > 0 && (--retry_budget) > 0);

  // Decoding must be total: every 16-bit word, including nonsense (a floating MISO
  // reads 0xFFFF), must map to a TINYSTEP_STATUS_* so tinystep_move()'s switch stays
  // exhaustive and its default: stays dead.
  return 0; // or a different error code here if appropriate
}

tinystep_result tinystep_move(
  tinystep *ts,
  int32_t pos,
  uint32_t max_polls,
  int *platform_error
) {
  TINYSTEP_ASSERT(ts != NULL);
  TINYSTEP_ASSERT(platform_error != NULL);
  TINYSTEP_ASSERT(ts -> is_configured);
  TINYSTEP_ASSERT(max_polls <= INT32_MAX);

  tinystep_result result = tinystep_async_move_start(ts, pos, platform_error);
  if (result != TINYSTEP_RESULT_OK)  return result;

  tinystep_result status = TINYSTEP_STATUS_BUSY;
  int32_t retry_budget = (int32_t) max_polls;
  do {
    do {
      *platform_error = tinystep_platform_wait_event(ts->event);
    } while (*platform_error > 0 && retry_budget --> 0);
    if (*platform_error > 0) return TINYSTEP_RESULT_TIMEOUT;
    else if (*platform_error < 0) return TINYSTEP_RESULT_PLATFORM_ERR;
    status = tinystep_async_move_status(ts, platform_error);
  } while (retry_budget --> 0 && status == TINYSTEP_STATUS_BUSY);

  switch (status) {
    case TINYSTEP_STATUS_DONE:
      return TINYSTEP_RESULT_OK;
    case TINYSTEP_STATUS_BUSY:
      return TINYSTEP_RESULT_TIMEOUT;
    case TINYSTEP_STATUS_ENCODER_FAULT:
      return TINYSTEP_RESULT_ENCODER_FAULT;
    case TINYSTEP_STATUS_PLATFORM_ERR:
      return TINYSTEP_RESULT_PLATFORM_ERR;
    default:
      TINYSTEP_ASSERT(false);
      return TINYSTEP_RESULT_BAD_ARG;
  }
}
