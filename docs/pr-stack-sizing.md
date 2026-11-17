# PR stack sizing handoff

Goal: make the open stack merge-ready. Another agent will do the trimming; this
note records where the lines are and what to cut.

Measured 2026-11-17 with `git diff <base>..<head>` additions; deletions in
parentheses. `test/**` = test, `src/**` = impl, everything else (docs, `info.md`,
workflows, scripts, prototypes) = other.

```
main
 └─ #2  interfaces            +897  (-220)  test 431  impl 214  other 252
     └─ #3  motion-planner   +557  (-5)    test 267  impl 202  other  88
         └─ #5  closed-loop  +1484 (-282)  test 635  impl 553  other 296
             └─ #6  1x1      +1584 (-149)  test 463  impl 670  other 451
                 └─ #7  shared-engine +979 (-89) test 402 impl 378 other 199
                     └─ #8  q8-area-test (draft) +3996 (-920)
                         test 321  impl 512  other 3163
                         └─ exp/p-only-fixed-kp +122 (-27)
                             test 18  impl 17  other 87   (no PR yet)
```

Total: **+9,619 / −1,692**; test 2,537, impl 2,546, other 4,536.

## Trim targets

- #2, #3, #5 are feature-sized and balanced; keep them, squash fixups only.
- #6 and #7 are impl-heavy; keep, but drop experiment commits and stale docs.
- #8 is the slop: 3,163 of its 3,996 added lines are docs/prototypes. Collapse
  the Q11/Q12/Q13 handoffs into one design note, delete superseded handoffs, and
  drop checked-in measurement JSON / generated LUT dumps that `scripts/` rebuilds.
- `exp/p-only-fixed-kp` is the 1x1-fit tip. Push its missing doc commit, fold the
  doc into the consolidated design note, and make this the final area PR.
- Target ≤400 impl lines per PR, or one clearly-scoped 4–6k impl PR for the area
  work. Logic PRs should keep test additions ≥ impl additions.
- Remove `EXPERIMENT` / `do not merge` / temporary branch notes before review.
- Each PR's docs must describe that PR's own RTL. Only the tip may claim 1x1 fit.

## Check

- `gh pr view N --json additions,deletions,changedFiles`
- `git diff --stat <base>..<head>`
- RTL: `pytest test/test_shared_controller.py -q` and `make -C test -B`
- Tip: CI `gds` must show 0 DRT violations and precheck pass.
