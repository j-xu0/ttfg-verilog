/* SPDX-License-Identifier: Apache-2.0 */
#include <stdio.h>

#include "tinystep.h"
#include "tinystep_platform_host.h"

static int failures;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            failures++;                                                \
        }                                                              \
    } while (0)

/* {1'b1, addr[1:0], payload[12:0]} */
#define WRITE(addr, payload) \
    (uint16_t)(0x8000u | ((uint16_t)(addr) << 13) | ((uint16_t)(payload) & 0x1FFFu))

static tinystep make(void) {
    tinystep_host_reset();
    tinystep dev = tinystep_init(tinystep_host_spi(),
                                 tinystep_host_isr_init(),
                                 tinystep_host_enable(),
                                 tinystep_host_event(),
                                 TINYSTEP_ACCEL_977);
    CHECK(tinystep_config(&dev) == 0);
    return dev;
}

int main(void) {
    int err = 0;
    int frames;

    /* Out of range is rejected before any frame is sent. */
    tinystep dev = make();
    frames = tinystep_host_transfers();

    CHECK(tinystep_async_move_start(&dev, TINYSTEP_MAX_POS + 1, &err) == TINYSTEP_RESULT_BAD_ARG);
    CHECK(tinystep_async_move_start(&dev, TINYSTEP_MIN_POS - 1, &err) == TINYSTEP_RESULT_BAD_ARG);
    CHECK(tinystep_host_transfers() == frames);

    /* Both limits encode as signed 13-bit. */
    CHECK(tinystep_async_move_start(&dev, TINYSTEP_MAX_POS, &err) == TINYSTEP_RESULT_OK);
    CHECK(tinystep_host_tx_at(frames) == WRITE(0, TINYSTEP_MAX_POS));
    CHECK(tinystep_host_target() == TINYSTEP_MAX_POS);

    CHECK(tinystep_async_move_start(&dev, TINYSTEP_MIN_POS, &err) == TINYSTEP_RESULT_OK);
    CHECK(tinystep_host_tx_at(frames + 1) == WRITE(0, TINYSTEP_MIN_POS));
    CHECK(tinystep_host_target() == TINYSTEP_MIN_POS);

    /* Enable drives the pin and arms the chip; disable aborts the move. */
    CHECK(!tinystep_host_enable_level());
    CHECK(tinystep_enable(&dev) == 0);
    CHECK(tinystep_host_enable_level());

    CHECK(tinystep_async_move_start(&dev, 32, &err) == TINYSTEP_RESULT_OK);
    CHECK(tinystep_host_busy());
    CHECK(tinystep_disable(&dev) == 0);
    CHECK(!tinystep_host_enable_level());
    CHECK(!tinystep_host_busy());

    /* A stalled chip never signals, so the poll budget runs out. */
    dev = make();
    CHECK(tinystep_enable(&dev) == 0);
    tinystep_host_set_stall(true);
    CHECK(tinystep_move(&dev, 64, 4, &err) == TINYSTEP_RESULT_TIMEOUT);
    CHECK(tinystep_host_busy());

    if (failures == 0) printf("ok: move command\n");
    return failures == 0 ? 0 : 1;
}
