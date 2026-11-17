# xltw-solver

MV2's solver for the algorithm track of CVRPLib's **VRPTW Challenge** (vehicle routing with time windows; large
XLTW instances, 1,000-10,000 customers). It is an offline, protocol-exact package of his vehicles-first baseline built
on [PyVRP](https://github.com/PyVRP/PyVRP).

MV2 of Munim, Inc. (mv2@munimtech.com). Made with AI agents working in MV2's name. Licence: Apache-2.0 (see
`LICENSE`). PyVRP 0.14.1 (MIT), numpy (BSD-3-Clause), vrplib (MIT) and tqdm (MPL-2.0 and MIT) keep their own licences.

**Status: v0.2.0, a working baseline, not yet a competitive entry.** The challenge opens registration and
submissions on 2026-12-16 and closes on 2027-01-16
([overview](https://galgos.inf.puc-rio.br/cvrplib/en/challenges/vrptw/overview)). Its rules ask for a solver that
builds with `make` or `build.sh` at the repository root, runs on Linux x86-64 without Internet access or root, is
called as `./solver INSTANCE SECONDS SEED`, is single-threaded, is deterministic for a given seed, and prints only
solutions on stdout.

## Build and run

```
python3 lock.py --resolve --fetch   # once, with Internet: downloads the pinned wheels into vendor/wheels/
bash build.sh --offline             # or: make offline   (plain `bash build.sh` / `make` may download missing wheels)
./solver INSTANCE SECONDS SEED      # deterministic for its seed (default, XLTW_MODE=work)
XLTW_MODE=clock ./solver INSTANCE SECONDS SEED   # 0.1.0's wall-clock stopping, not deterministic
```

Needs Linux x86-64 (glibc 2.28 or newer), bash and CPython 3.11, 3.12 or 3.13. No pip, no venv, no root.
`lock.py` unpacks pinned wheels (pyvrp 0.14.1, numpy 2.3.4, vrplib 2.2.0, tqdm 4.67.1), each checked against the
sha256 PyPI publishes for it (`wheels.lock`), into `lib/`. `build.sh` uses `XLTW_PYTHON` if set, else the default
`python3`, records the interpreter in `lib/XLTW_BUILD.json`, and writes the `./solver` launcher that always runs it.

The wheels are **not** in this repository yet. The challenge's build machine has no Internet, so before submitting,
either `vendor/wheels/` gets committed (about 20 MB for one Python version) or the organisers confirm another way.
That decision is open.

## What the solver does

- Nearest-integer Euclidean arcs (the challenge's convention).
- A fleet lower bound (`xltwlb.py`: capacity, bin-packing and time-window bounds).
- PyVRP's fleet-reduction loop toward that bound, then distance search with the best fleet found. The challenge
  ranks fewer vehicles first, distance second. Since 0.2.0 the fleet phase gets 0.8 of the work (was 0.5): in
  `xltwff/` that saved one vehicle of 166 at most 0.33% extra distance.
- **Deterministic for its seed (0.2.0).** Each phase stops after a number of PyVRP iterations fixed in advance from
  the number of customers n and the limit T only, never from the machine:
  fleet `floor(90 x g(n) x 0.8 x usable)`, distance `floor(200 x g(n) x 0.2 x usable)` plus the fleet iterations the
  fleet phase did not need (at 200/90 each), with `g(n) = (n/1000)^-0.5` and `usable = T - margin(T)`. The rates
  (90 and 200 iterations per second at n = 1,000) are about two thirds of the slowest per-phase speeds measured in
  [`xltwdet/`](https://github.com/munimtechnologies/cvrplib-baselines/tree/main/xltwdet), so a slower machine still
  finishes in time; the cost is that a fast machine leaves part of the limit unused. The wall clock is only a
  safety net, and the stats line on stderr says when it acted (`clock_stop`, `deterministic`).
- Every better solution is printed in CVRPLib format (`Route #k: ...` lines and a `Cost: N` line). Before importing
  anything heavy, `solver.py` keeps a private copy of stdout for solutions and points fd 1 at stderr, so library
  logging and stray prints can only reach stderr.
- It stops by itself before the limit. A watchdog thread also ends it on time, or at once on SIGTERM, even while the
  main thread is inside a long PyVRP C++ call (PyVRP's search releases the GIL).

Known gaps, in the order we plan to close them:

1. **Determinism at the full limit is pre-registered, not yet confirmed.** `xltwdet/` showed that an iteration
   budget makes a seed repeat its answer (3 of 3 identical runs on each of three instances at 120 s), but its single
   budget for both phases was too tight for one slow fleet phase. 0.2.0's separate budgets are tested at the full
   limit T(n) on two instances, with two copies running side by side, in
   [xltwdet2/](https://github.com/munimtechnologies/cvrplib-baselines/tree/main/xltwdet2) (predictions published
   before the run).
2. **The fleet phase is weak** on capacity-free instances: at the full limit T(n) it still ends 26-39% above a
   lower bound on vehicles (`xltwfull/` in cvrplib-baselines); more time, a fixed-cost search (`xltwfc/`) and a
   dedicated route minimiser (`nbrm/`) barely help. Whether the gap is the search's or the bound's is being tested
   from both sides: a fixed-time matching fleet search (`tmatch/`) and a column-generation fleet bound (`cgfleet/`),
   both pre-registered in [cvrplib-baselines](https://github.com/munimtechnologies/cvrplib-baselines).
3. Wheels and a method note for the submission.

## Tests

- `python3 -m unittest -v test_pkg` (21 tests; the ones that need `lib/` skip until `build.sh` has run).
- `python3 -m unittest -v test_budget` (0.2.0's work budget; standard library only).
- `python3 conformance.py --data-dir DATA --out-dir OUT` runs `./solver` the way the challenge does (SIGTERM at the
  limit, SIGKILL 5 s later, stdout read line by line with arrival times) on six short XLTW-Test runs, re-checks every
  counted solution exactly, and flags runs the host paused (a gap of more than 1 s between the harness's polls).

The conformance test was pre-registered (`expectations.md`, `addendum-2026-11-06.md`) and scored on 2026-11-06 for
0.1.0: offline build, clean stdout, feasible solutions with exact `Cost` lines, self-stop before the limit, at least
0.5 s of slack and under 0.6 GB on python:3.11 and python:3.12, and one 8,574-customer run at T(n)/30 that stopped by
itself. Full score, including the two predictions that failed:
[xltwpkg/score.md](https://github.com/munimtechnologies/cvrplib-baselines/blob/main/xltwpkg/score.md).
One comment in `conformance.py` and the "why" section of `expectations.md` blame an earlier failure on a stuck
PyVRP call; that turned out to be a host pause
([pause-audit.md](https://github.com/munimtechnologies/cvrplib-baselines/blob/main/xltwrun/pause-audit.md)).
The files are kept byte-for-byte as tested.

## Credits

PyVRP: Wouda, Lan and Kool, "PyVRP: a high-performance VRP solver package", INFORMS Journal on Computing (2024).
Fleet-bound ideas: Kontoravdis and Bard (1995); Afifi, Guibadj and Moukrim (2014). XLTW instances and the challenge:
CVRPLib (https://galgos.inf.puc-rio.br/cvrplib/).
