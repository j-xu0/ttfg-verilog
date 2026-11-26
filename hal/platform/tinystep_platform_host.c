/* SPDX-License-Identifier: Apache-2.0 */
#include "tinystep_platform_host.h"

/* Status word bits, from stepper_spi.v. */
#define STATUS_BUSY    (1u << 14)
#define STATUS_AT_TGT  (1u << 13)
#define STATUS_FAULT   (1u << 12)
#define STATUS_RANGE   (1u << 1)
#define STATUS_ENCODER (1u << 0)

/* Register addresses. */
#define REG_TARGET 0u
#define REG_RESERVED 1u
#define REG_PLAN 2u
#define REG_ZERO 3u

/* Fixed 4 steps/tick ceiling, plus start/stop overhead. */
#define VMAX_STEPS_PER_TICK 4
#define MOVE_OVERHEAD_TICKS 2

#define FRAME_LOG 64

/* Opaque handle types from tinystep.h. */
struct tinystep_platform_spi_s { int unused; };
struct tinystep_platform_isr_init_s { int unused; };
struct tinystep_platform_gpio_s { int unused; };
struct tinystep_platform_event_s { int unused; };

static struct tinystep_platform_spi_s s_spi;
static struct tinystep_platform_isr_init_s s_isr_init;
static struct tinystep_platform_gpio_s s_enable;
static struct tinystep_platform_event_s s_event;

static struct {
    int16_t target;
    int16_t position;
    uint8_t plan;
    bool busy;
    bool at_target;
    bool range_fault;
    bool encoder_fault;
    bool motion_enabled;
    bool stall;
    bool event_posted;
    uint32_t ticks_left;
    tinystep_isr isr;
    tinystep *isr_ctx;
} chip;

static uint16_t frame_log[FRAME_LOG];
static int frame_count;
static int ticks;

static void raise_int(void) {
    chip.event_posted = true;
    if (chip.isr != NULL) chip.isr(chip.isr_ctx);
}

static void arrive(void) {
    chip.position = chip.target;
    chip.busy = false;
    chip.at_target = true;
    chip.ticks_left = 0;
    raise_int();
}

static void start_move(void) {
    int delta = (int)chip.target - (int)chip.position;
    unsigned distance = (unsigned)(delta < 0 ? -delta : delta);

    chip.busy = true;
    chip.at_target = false;
    chip.ticks_left = distance / VMAX_STEPS_PER_TICK + MOVE_OVERHEAD_TICKS;
}

static void advance_tick(void) {
    if (!chip.busy || chip.stall) return;

    ticks++;
    if (chip.ticks_left > 0) chip.ticks_left--;
    if (chip.ticks_left == 0) arrive();
}

static uint16_t status_word(void) {
    uint16_t word = 0;

    if (chip.busy) word |= STATUS_BUSY;
    if (chip.at_target) word |= STATUS_AT_TGT;
    if (chip.range_fault) word |= STATUS_RANGE;
    if (chip.encoder_fault) word |= STATUS_ENCODER;
    if (chip.range_fault || chip.encoder_fault) word |= STATUS_FAULT;

    return word;
}

int tinystep_platform_xfer16(tinystep_platform_spi *spi, uint16_t tx, uint16_t *rx) {
    (void)spi;

    if (frame_count < FRAME_LOG) frame_log[frame_count] = tx;
    frame_count++;

    if ((tx & 0x8000u) == 0) {
        if (rx != NULL) *rx = status_word();
        /* Any read acknowledges the sticky faults. */
        chip.range_fault = false;
        chip.encoder_fault = false;
        return 0;
    }

    /* Writes during a move are silently dropped, as the RTL does. */
    if (chip.busy) return 0;

    uint16_t payload = (uint16_t)(tx & 0x1FFFu);
    int value = (int)payload;
    if ((value & 0x1000) != 0) value -= 0x2000;

    switch ((tx >> 13) & 3u) {
    case REG_TARGET:
        chip.target = (int16_t)value;
        if (chip.motion_enabled) start_move();
        break;
    case REG_RESERVED:
        break;
    case REG_PLAN:
        chip.plan = (uint8_t)payload;
        chip.plan &= 0x7u;
        break;
    case REG_ZERO:
        if (payload == 0 && !chip.motion_enabled) {
            chip.position = 0;
            chip.target = 0;
        }
        break;
    default:
        break;
    }

    return 0;
}

int tinystep_platform_register_isr(tinystep_platform_isr_init *isr_init,
                                   tinystep *ts,
                                   tinystep_isr isr) {
    (void)isr_init;
    chip.isr = isr;
    chip.isr_ctx = ts;
    return 0;
}

int tinystep_platform_write_gpio(tinystep_platform_gpio *gpio, bool value) {
    (void)gpio;
    chip.motion_enabled = value;
    if (!value) {
        /* Motion enable low aborts the active command. */
        chip.busy = false;
        chip.at_target = false;
        chip.ticks_left = 0;
    }
    return 0;
}

int tinystep_platform_post_event(tinystep_platform_event *event) {
    (void)event;
    chip.event_posted = true;
    return 0;
}

int tinystep_platform_wait_event(tinystep_platform_event *event) {
    (void)event;

    advance_tick();

    if (chip.event_posted) {
        chip.event_posted = false;
        return 0;
    }

    /* Would block: positive means retryable, so the driver spends a poll. */
    return 1;
}

tinystep_platform_spi *tinystep_host_spi(void) { return &s_spi; }
tinystep_platform_isr_init *tinystep_host_isr_init(void) { return &s_isr_init; }
tinystep_platform_gpio *tinystep_host_enable(void) { return &s_enable; }
tinystep_platform_event *tinystep_host_event(void) { return &s_event; }

void tinystep_host_reset(void) {
    chip.target = 0;
    chip.position = 0;
    chip.plan = 0;
    chip.busy = false;
    chip.at_target = false;
    chip.range_fault = false;
    chip.encoder_fault = false;
    chip.motion_enabled = false;
    chip.stall = false;
    chip.event_posted = false;
    chip.ticks_left = 0;
    chip.isr = NULL;
    chip.isr_ctx = NULL;
    frame_count = 0;
    ticks = 0;
}

int tinystep_host_transfers(void) { return frame_count; }

uint16_t tinystep_host_tx_at(int index) {
    if (index < 0 || index >= frame_count || index >= FRAME_LOG) return 0;
    return frame_log[index];
}

bool tinystep_host_busy(void) { return chip.busy; }
bool tinystep_host_at_target(void) { return chip.at_target; }
bool tinystep_host_fault_latched(void) { return chip.range_fault || chip.encoder_fault; }
int16_t tinystep_host_target(void) { return chip.target; }
int16_t tinystep_host_position(void) { return chip.position; }
uint8_t tinystep_host_plan(void) { return chip.plan; }
bool tinystep_host_enable_level(void) { return chip.motion_enabled; }
int tinystep_host_ticks(void) { return ticks; }

void tinystep_host_set_stall(bool stalled) { chip.stall = stalled; }

void tinystep_host_inject_fault(bool range_fault, bool encoder_fault) {
    chip.range_fault = chip.range_fault || range_fault;
    chip.encoder_fault = chip.encoder_fault || encoder_fault;
    chip.busy = false;
    chip.at_target = false;
    raise_int();
}
