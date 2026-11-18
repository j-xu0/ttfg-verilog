/* SPDX-License-Identifier: Apache-2.0 */
#include <stdio.h>

#include "tinystep.h"
#include "tinystep_platform_host.h"

/* Bring-up sketch against the simulated host platform. Prints, does not check. */
int main(void)
{
    tinystep_host_reset();

    tinystep ts = tinystep_init(tinystep_host_spi(),
                                tinystep_host_isr_init(),
                                tinystep_host_enable(),
                                tinystep_host_event(),
                                TINYSTEP_ACCEL_977);

    printf("config:  %d\n", tinystep_config(&ts));
    printf("enable:  %d\n", tinystep_enable(&ts));
    printf("disable: %d\n", tinystep_disable(&ts));

    return 0;
}
