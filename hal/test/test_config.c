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

int main(void) {
    tinystep_host_reset();

    /* A fault latched before config must be acknowledged by config's read. */
    tinystep_host_inject_fault(false, true);
    CHECK(tinystep_host_fault_latched());

    tinystep dev = tinystep_init(tinystep_host_spi(),
                                tinystep_host_isr_init(),
                                tinystep_host_enable(),
                                tinystep_host_event(),
                                TINYSTEP_ACCEL_4883);

    CHECK(tinystep_config(&dev) == 0);

    /* Two frames: the PLAN write, then a read that acks the fault. */
    CHECK(tinystep_host_transfers() == 2);
    CHECK(tinystep_host_tx_at(0) == WRITE(2, TINYSTEP_ACCEL_4883));
    CHECK((tinystep_host_tx_at(1) & 0x8000u) == 0);
    CHECK(!tinystep_host_fault_latched());
    CHECK(tinystep_host_plan() == TINYSTEP_ACCEL_4883);

    /* Configure is idempotent: no additional frames. */
    CHECK(tinystep_config(&dev) == 0);
    CHECK(tinystep_host_transfers() == 2);

    /* A clean chip still gets its ack read, and the plan is applied. */
    tinystep_host_reset();
    CHECK(!tinystep_host_fault_latched());
    tinystep second = tinystep_init(tinystep_host_spi(),
                                   tinystep_host_isr_init(),
                                   tinystep_host_enable(),
                                   tinystep_host_event(),
                                   TINYSTEP_ACCEL_100098);
    CHECK(tinystep_config(&second) == 0);
    CHECK(tinystep_host_tx_at(0) == WRITE(2, TINYSTEP_ACCEL_100098));
    CHECK(tinystep_host_plan() == TINYSTEP_ACCEL_100098);

    if (failures == 0) printf("ok: config\n");
    return failures == 0 ? 0 : 1;
}
