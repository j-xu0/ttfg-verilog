/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TINYSTEP_ASSERT_H
#define TINYSTEP_ASSERT_H

/*
 * TINYSTEP_ASSERT(cond)          contract violation, fixed in source
 * TINYSTEP_ASSERT_MSG(cond, msg) same, with a message
 * TINYSTEP_STATIC_ASSERT(c,name) compile-time equivalent
 *
 * Contract violations only: NULL handles, wrong call order, out-of-range
 * compile-time constants. Never on wire/sensor data -- return an error.
 *
 * Independent of NDEBUG and <assert.h> on purpose: a check that disappears
 * in the release build is worse than no check.
 *
 * Override before the first include, or via -D. Choose any one:
 *   define TINYSTEP_ASSERT(cond)           function-like, e.g.
 *                                          -D'TINYSTEP_ASSERT(x)=my_trap(#x)'
 *   define TINYSTEP_ASSERT_MSG(cond, msg)  function-like
 *   define TINYSTEP_ASSERT_DISABLE         compile both out (cond not evaluated)
 * Also settable: TINYSTEP_ASSERT_TRAP(), TINYSTEP_ASSERT_REPORT(cond, msg),
 * TINYSTEP_ASSERT_UNREACHABLE(), or TINYSTEP_ASSERT_VERBOSE for stderr output.
 *
 * cond is evaluated exactly once; keep side effects out of it.
 */

/* --- trap ------------------------------------------------------------- */
#ifndef TINYSTEP_ASSERT_TRAP
#  if defined(__GNUC__) || defined(__clang__)
#    define TINYSTEP_ASSERT_TRAP() __builtin_trap()
#  elif defined(_MSC_VER)
#    define TINYSTEP_ASSERT_TRAP() __debugbreak()
#  else
#    include <stdlib.h>
#    define TINYSTEP_ASSERT_TRAP() abort()
#  endif
#endif

/* Tell the optimizer control does not continue past a failed assert. */
#ifndef TINYSTEP_ASSERT_UNREACHABLE
#  if defined(__GNUC__) || defined(__clang__)
#    define TINYSTEP_ASSERT_UNREACHABLE() __builtin_unreachable()
#  elif defined(_MSC_VER)
#    define TINYSTEP_ASSERT_UNREACHABLE() __assume(0)
#  else
#    define TINYSTEP_ASSERT_UNREACHABLE() ((void)0)
#  endif
#endif

/* --- optional reporting ----------------------------------------------- */
#ifndef TINYSTEP_ASSERT_REPORT
#  ifdef TINYSTEP_ASSERT_VERBOSE
#    include <stdio.h>
#    define TINYSTEP_ASSERT_REPORT(cond, msg) \
       fprintf(stderr, "%s:%d: tinystep: %s: %s\n", __FILE__, __LINE__, (msg), (cond))
#  else
#    define TINYSTEP_ASSERT_REPORT(cond, msg) ((void)0)
#  endif
#endif

/* --- resolution ------------------------------------------------------- */
#if defined(TINYSTEP_ASSERT_DISABLE)

#  define TINYSTEP_ASSERT_MSG(cond, msg) ((void)0)
#  define TINYSTEP_ASSERT(cond)          ((void)0)

#elif defined(TINYSTEP_ASSERT) && defined(TINYSTEP_ASSERT_MSG)
/* both supplied by the user */
#elif defined(TINYSTEP_ASSERT)

#  define TINYSTEP_ASSERT_MSG(cond, msg) TINYSTEP_ASSERT(cond)
#elif defined(TINYSTEP_ASSERT_MSG)

#  define TINYSTEP_ASSERT(cond) TINYSTEP_ASSERT_MSG(cond, "assertion failed")
#else

#  define TINYSTEP_ASSERT_MSG(cond, msg)   \
     do {                                   \
       if (!(cond)) {                       \
         TINYSTEP_ASSERT_REPORT(#cond, (msg)); \
         TINYSTEP_ASSERT_TRAP();            \
         TINYSTEP_ASSERT_UNREACHABLE();     \
       }                                    \
     } while (0)
#  define TINYSTEP_ASSERT(cond) TINYSTEP_ASSERT_MSG(cond, "assertion failed")
#endif

/* --- compile-time companion ------------------------------------------- */
#ifndef TINYSTEP_STATIC_ASSERT
#  if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 201112L)
#    define TINYSTEP_STATIC_ASSERT(cond, name) \
       typedef char tinystep_static_assert_##name[(cond) ? 1 : -1]
#  else
#    define TINYSTEP_STATIC_ASSERT(cond, name) _Static_assert(cond, #name)
#  endif
#endif

#endif /* TINYSTEP_ASSERT_H */
