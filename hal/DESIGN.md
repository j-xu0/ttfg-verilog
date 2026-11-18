# Design decisions

Choices that recur in review. Settle them here, not in a diff.

**The driver struct is not opaque.** Opaque needs caller-provided storage or an allocator, both
more machinery than this driver earns. Fields are documented as off-limits, and `tinystep` must
not be moved or copied after `tinystep_config()`, because the ISR holds its address.

**`tinystep_init` validates and cannot fail.** It returns the struct, so its checks are asserts
by design; an error return would reopen the non-opaque decision. The `& 0x7` on the PLAN payload
is blast-radius protection, not the check.

**Interrupts before `config()` completes are dropped on purpose.** `is_configured` is set last.
That window loses at most one edge, and the alternative is handing the ISR a half-built driver.

**The platform is link-time, not a vtable.** Declarations in a header, one definition per target
in `platform/<target>.c`, chosen at link. A partial port is a link error rather than a NULL call
in an ISR, and the call is direct on the interrupt path. A vtable would buy runtime selection or
two ports in one binary; neither is needed.

**The core owns retry policy; the port reports.** Port functions return zero, negative for
non-retryable, positive for retryable; backoff and budget live in `src/` so every target behaves
identically. The port also owns CS, so the core never toggles it and a frame stays atomic against
an interrupt-induced read.

**No SPI in the ISR.** The ISR only posts the event; every transfer happens in thread context.
That is why `wait_event` is a port primitive, and why bus errors never surface from interrupt
context.

**Events coalesce.** The port's event is binary: several events before a service leave one token.
Costs a few polls bounded by `max_polls`; saves a counter, a drain and a re-arm path.

**Status and result are separate namespaces.** `TINYSTEP_STATUS_*` is what the chip reported,
`TINYSTEP_RESULT_*` is how the call ended; never assign one to the other, as their numeric
overlap already caused a real bug. The decode is total — every 16-bit word maps to a status —
which keeps `move()`'s `default:` dead.

**Asserts are for source bugs, error codes for data.** Asserts: NULL handles, calling before
`config()`, out-of-range compile-time constants such as `max_accel`. Error codes: values a caller
may legitimately hold (an out-of-range `pos`) and anything from the environment (bus errors,
exhausted retries). Never clamp a position; never assert on wire or sensor data.
