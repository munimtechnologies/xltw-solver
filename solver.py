#!/usr/bin/env python3
"""xltw-solver 0.2.0: a vehicles-first PyVRP solver packaged for CVRPLib's VRPTW Challenge (algorithm track).

MV2 of Munim, Inc. (mv2@munimtech.com). Made with AI agents working in MV2's name. Apache-2.0.
Uses PyVRP 0.14.1 (MIT) for the search. The method is the `xltwrun/solver.py` baseline of
munimtechnologies/cvrplib-baselines (commit ba1eec8); this file adds what an offline, protocol-exact package needs:

- It finds its own code and its unpacked dependencies (`lib/`, made by build.sh) without pip, a venv or a network.
- Its stdout carries solutions only. At start it keeps a private copy of file descriptor 1 for solutions and points
  fd 1 and `sys.stdout` at stderr, so library logging (PyVRP attaches a log handler to stdout), stray prints and
  C-level writes all land on stderr.
- A watchdog thread ends the process on time even if the search is stuck inside one long C++ call (see Watchdog):
  at a hard deadline just before the limit, or at once on SIGTERM, it prints any held better solution and exits 0.

Usage (as the challenge runs algorithms): ./solver INSTANCE SECONDS SEED

- Reads a VRPLIB VRPTW instance (one depot). Distances and travel times are Euclidean, rounded to the nearest
  integer per arc, floor(d + 0.5), as the challenge's rules say.
- Fleet lower bound K = max(Martello-Toth L2, time-window clique) from `xltwlb.py`, given at most 10% of the limit;
  if that is not enough, the capacity bound ceil(total demand / capacity) is used instead.
- Fleet phase: PyVRP's minimise_fleet loop (one vehicle fewer, solve to the first feasible solution, shrink to the
  routes used, repeat), stopped at K.
- Distance phase: PyVRP's `solve` with that fleet, started from the fleet phase's solution.
- Every better solution (fewer routes first, then shorter) is printed in CVRPLib format, at most once per
  max(1 s, 1% of the limit); the best one is always printed before the solver stops. Nothing else goes to stdout.
- The watchdog's hard deadline is half the margin (1.5 s + 0.2% of the limit, at most 30 s) before the limit, and
  SIGTERM ends the process at once.

0.2.0 (2026-11-17): deterministic for its seed by default. The challenge's rules say "Each execution must be
deterministic for a given seed". 0.1.0 stopped both phases on the wall clock, so the same seed could end at different
solutions on a faster, slower or busier machine. 0.2.0 stops each phase on a number of PyVRP iterations fixed in
advance from the only two inputs that are the same on every machine, the number of customers n and the limit T
(XLTW_MODE=work, the default):

    usable           = T - margin(T)
    fleet budget     = floor(RF0 x g(n) x FLEET_FRAC x usable)          g(n) = (n / 1000) ** -0.5
    distance budget  = floor(RD0 x g(n) x (1 - FLEET_FRAC) x usable)  + floor(unused fleet iterations x RD0 / RF0)

RF0 = 90 and RD0 = 200 iterations per second at n = 1,000 are deliberately low: in `xltwdet/` (cvrplib-baselines,
2026-11-07) the fleet phase ran at 125-350 it/s and the distance phase at 297-500 it/s at n = 1,047-1,233 alone, and
at 116 and 250 it/s under a CPU hog; scaled by g(n), the slowest were 129 (fleet) and 278 (distance). One rate for
both phases (det2 there) left too little room on the slow fleet phase of n1234; separate rates fix that. The wall
clock stays only as a safety net: if a phase reaches the usual stopping point first, it stops there, and the stats
line says so (`clock_stop`), because from then on the run is no longer deterministic.
FLEET_FRAC is 0.8 (was 0.5): in `xltwff/` 0.8 of the limit for the fleet phase saved one vehicle of 166 on three
capacity-free instances, and cost at most 0.33% distance where the counts were equal.
XLTW_MODE=clock restores 0.1.0's wall-clock stopping (with FLEET_FRAC 0.8), e.g. for runs without a time limit
protocol where determinism does not matter.

Single-threaded: PyVRP's search runs on one thread, and BLAS thread pools are pinned to one thread.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE, os.path.join(HERE, "lib")):  # lib/ ends up first, ahead of any system site-packages
    if _p not in sys.path:
        sys.path.insert(0, _p)
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import json  # noqa: E402
import math  # noqa: E402
import select  # noqa: E402
import signal  # noqa: E402
import tempfile  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

START = time.monotonic()
VERSION = "0.2.0"
CONV = "round"
BOUND_FRAC = float(os.environ.get("XLTW_BOUND_FRAC", "0.10"))
FLEET_FRAC = 0.8
RF0 = 90.0     # fleet-phase iterations per second at n = 1,000 (a low estimate)
RD0 = 200.0    # distance-phase iterations per second at n = 1,000 (a low estimate)
RATE_EXP = 0.5
MODES = ("work", "clock")
TERM = {"flag": False}
OUT = None  # the real stdout once claim_stdout() has run
B = None    # xltwbase, loaded by load_helpers()
X = None    # xltwlb


def claim_stdout():
    """Keep fd 1 for solutions only. Returns a text stream on a private copy of the original stdout; afterwards
    fd 1 and sys.stdout both point at stderr."""
    global OUT
    sys.stdout.flush()
    fd = os.dup(1)
    os.dup2(2, 1)
    OUT = os.fdopen(fd, "w", encoding="ascii", newline="\n")
    sys.stdout = sys.stderr
    return OUT


def load_helpers():
    global B, X
    if B is None:
        import xltwbase
        import xltwlb
        B, X = xltwbase, xltwlb
    return B, X


def margin(limit):
    """How long before the limit the solver stops by itself."""
    return min(1.5 + 0.002 * limit, 30.0)


def mode_from_env(env=None):
    env = os.environ if env is None else env
    m = env.get("XLTW_MODE", "") or "work"
    if m not in MODES:
        raise ValueError("XLTW_MODE must be one of %s" % ", ".join(MODES))
    return m


def size_factor(n):
    """g(n) = (n / 1000) ** -RATE_EXP: PyVRP's iterations per second fall roughly with the square root of n."""
    return (max(int(n), 1) / 1000.0) ** (-RATE_EXP)


def phase_budgets(n, limit):
    """(fleet, distance) iteration budgets for n customers and limit `limit` seconds. Depends on n and the limit
    only, never on the machine."""
    usable = max(float(limit) - margin(float(limit)), 0.0)
    g = size_factor(n)
    fleet = int(math.floor(RF0 * g * FLEET_FRAC * usable))
    dist = int(math.floor(RD0 * g * (1.0 - FLEET_FRAC) * usable))
    return fleet, dist


def carry(fleet_left):
    """Distance iterations bought by fleet iterations the fleet phase did not use."""
    return int(math.floor(max(int(fleet_left), 0) * RD0 / RF0))


def log(msg):
    sys.stderr.write("[%8.1fs] %s\n" % (time.monotonic() - START, msg))
    sys.stderr.flush()


def _on_term(signum, frame):  # noqa: ARG001
    TERM["flag"] = True


class Until:
    """PyVRP stopping criterion: an absolute monotonic deadline, or SIGTERM."""

    def __init__(self, deadline):
        self.deadline = deadline

    def __call__(self, best_cost):  # noqa: ARG002
        return TERM["flag"] or time.monotonic() >= self.deadline


class Budget:
    """PyVRP stopping criterion that counts iterations: stops when `left` reaches 0 (deterministic), and as a
    safety net at the wall-clock `deadline` or on SIGTERM (then `clock_stop` / `term_stop` is set)."""

    def __init__(self, left, deadline, clock=time.monotonic):
        self.left, self.used, self.deadline, self.clock = int(left), 0, deadline, clock
        self.clock_stop = self.term_stop = False

    def __call__(self, best_cost):  # noqa: ARG002
        if TERM["flag"]:
            self.term_stop = True
            return True
        if self.clock() >= self.deadline:
            self.clock_stop = True
            return True
        if self.left <= 0:
            return True
        self.left -= 1
        self.used += 1
        return False

    def exhausted(self):
        return self.left <= 0 or self.clock_stop or self.term_stop


class _Timeout(BaseException):
    """Raised by SIGALRM inside the bound. BaseException, so no `except Exception` in the bound code catches it."""


def capacity_bound(inst):
    total = sum(r[2] for r in inst["rows"][1:])
    return max(1, int(math.ceil(total / float(inst["capacity"]))))


def bound(inst, cap_seconds):
    """K in the challenge's convention with a time cap. Falls back to the capacity bound if the cap is hit."""
    kc = capacity_bound(inst)
    out = {"kc": kc, "kb": None, "kt": None, "k": kc, "timed_out": False, "seconds": 0.0}
    if cap_seconds <= 0:
        out["timed_out"] = True
        return out

    def _alarm(signum, frame):  # noqa: ARG001
        raise _Timeout()

    old = signal.signal(signal.SIGALRM, _alarm)
    t = time.monotonic()
    signal.setitimer(signal.ITIMER_REAL, cap_seconds)
    try:
        b = B.lower_bound(inst, CONV)
        k = b["k"] if b.get("clique_verified") is not False else b["kb"]
        out.update(kb=b["kb"], kt=b["kt"], k=max(int(k), kc))
    except _Timeout:
        out["timed_out"] = True
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)
    out["seconds"] = round(time.monotonic() - t, 2)
    return out


def format_solution(routes, cost):
    lines = ["Route #%d: %s" % (i + 1, " ".join(map(str, r))) for i, r in enumerate(routes)]
    lines.append("Cost: %d" % cost)
    return "\n".join(lines) + "\n"


def solution_routes(sol):
    """Customer numbers 1..n of every route of a PyVRP 0.14 solution."""
    return [B.route_clients(r) for r in sol.routes()]


class Printer:
    """Prints strictly better solutions (fewer routes, then lower cost), throttled to one print per `gap` seconds.
    A better solution that arrives within the gap is held and printed at the next chance or at `flush(True)`."""

    def __init__(self, limit, out=None, convert=solution_routes, clock=time.monotonic):
        self.gap = max(1.0, 0.01 * limit)
        self.out = out if out is not None else (OUT if OUT is not None else sys.stdout)
        self.convert = convert
        self.clock = clock
        self.key = None       # (routes, cost) of the last printed solution
        self.pending = None   # (key, solution) better than the last printed one, not printed yet
        self.last = None
        self.prints = 0
        self.lock = threading.RLock()  # the watchdog thread may flush too

    def best_key(self):
        return self.pending[0] if self.pending else self.key

    def offer(self, sol):
        if not sol.is_feasible():
            return False
        key = (int(sol.num_routes()), int(round(sol.distance())))
        with self.lock:
            best = self.best_key()
            if best is not None and key >= best:
                return False
            self.pending = (key, sol)
            self.flush(False)
        return True

    def flush(self, force):
        with self.lock:
            if self.pending is None:
                return False
            now = self.clock()
            if not force and self.last is not None and now - self.last < self.gap:
                return False
            key, sol = self.pending
            self.out.write(format_solution(self.convert(sol), key[1]))
            self.out.flush()
            self.key, self.pending, self.last = key, None, now
            self.prints += 1
            return True


class Watchdog:
    """Ends the process on time even when the search is stuck inside one long C++ call.

    Seen on XLTW-Test-n8575 at T/30 (cvrplib-baselines xltwrun part b2): after 64 s one PyVRP call ran past the
    limit, the solver never reached its own stopping check, and SIGTERM could not help, because a Python signal
    handler only runs when the main thread returns to Python. PyVRP's local search and perturbation release the
    GIL, so this thread still runs then. It wakes at `hard_deadline` (after the solver's own stopping point, before
    the limit) or as soon as SIGTERM arrives (through signal.set_wakeup_fd, whose byte is written by the C-level
    handler at once), prints any held better solution, flushes stdout and exits with code 0."""

    def __init__(self, printer, hard_deadline, out):
        self.printer, self.hard, self.out = printer, hard_deadline, out
        self.r, self.w = os.pipe()
        os.set_blocking(self.r, False)
        os.set_blocking(self.w, False)
        signal.set_wakeup_fd(self.w)  # main thread only
        self.thread = threading.Thread(target=self._loop, name="xltw-watchdog", daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _loop(self):
        why = "deadline"
        while True:
            left = self.hard - time.monotonic()
            if left <= 0:
                break
            ready, _, _ = select.select([self.r], [], [], min(left, 0.5))
            if ready:
                try:
                    data = os.read(self.r, 512)
                except (BlockingIOError, InterruptedError):
                    data = b""
                if int(signal.SIGTERM) in data:
                    why = "sigterm"
                    break
        self.fire(why)

    def fire(self, why):
        try:
            with self.printer.lock:
                self.printer.flush(True)
                self.out.flush()
        finally:
            sys.stderr.write("XLTW_WATCHDOG %s at %.2fs, prints %d\n" % (why, time.monotonic() - START,
                                                                          self.printer.prints))
            sys.stderr.flush()
            os._exit(0)


def read_data(inst):
    """PyVRP data in the challenge's convention, from the parsed instance (depot first)."""
    import pyvrp
    fd, path = tempfile.mkstemp(suffix=".vrp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(B.to_vrplib(inst))
        return pyvrp.read(path, round_func=B.rf_round)
    finally:
        os.remove(path)


def run(path, limit, seed, out=None, watch=True, mode=None):
    """Solve one instance under the protocol. Returns the stats dict (also logged to stderr)."""
    load_helpers()
    from pyvrp import IteratedLocalSearchCallbacks, IteratedLocalSearchParams, Solution, SolveParams, solve
    from pyvrp.stop import FirstFeasible, MultipleCriteria

    mode = mode or mode_from_env()
    deadline = START + limit - margin(limit)
    seed = int(seed) % (2 ** 31)
    with open(path, encoding="utf-8") as f:
        inst = X.parse_any(f.read())
    n = len(inst["rows"]) - 1
    pr = Printer(limit, out=out)
    if watch:
        Watchdog(pr, START + limit - margin(limit) / 2.0, pr.out).start()
    b = bound(inst, BOUND_FRAC * limit)
    log("bound %s" % json.dumps(b))
    data = read_data(inst)
    fleet_budget, dist_budget = phase_budgets(n, limit)
    stats = {"solver": "xltw-solver " + VERSION, "instance": inst["name"], "customers": n, "limit": limit,
             "seed": seed, "mode": mode, "bound": b, "fleet_frac": FLEET_FRAC, "fleet_solves": 0,
             "fleet_iterations": 0, "iterations": 0, "clock_stop": False, "term_stop": False}
    if mode == "work":
        g = size_factor(n)
        stats.update({"fleet_budget": fleet_budget, "distance_budget": dist_budget,
                      "model_fleet_rate": round(RF0 * g, 2), "model_distance_rate": round(RD0 * g, 2)})
        log("work budgets: fleet %d, distance %d iterations" % (fleet_budget, dist_budget))

    # Fleet phase: minimise_fleet's loop, stopped at K. `cur` always holds the data with the last feasible fleet,
    # so at most two copies of the matrices exist at a time.
    t_fleet = time.monotonic()
    stats["pre_seconds"] = round(t_fleet - START, 2)
    if mode == "work":
        fstop = Budget(fleet_budget, deadline)
        fleet_open = lambda: not fstop.exhausted()  # noqa: E731
    else:
        fleet_deadline = t_fleet + FLEET_FRAC * max(deadline - t_fleet, 0.0)
        fstop = Until(fleet_deadline)
        fleet_open = lambda: time.monotonic() < fleet_deadline  # noqa: E731
    cur, feas, best = data, data.vehicle_type(0), None
    del data
    while feas.num_available > b["k"] and not TERM["flag"] and fleet_open():
        fleet = feas.replace(num_available=feas.num_available - 1)
        trial = cur.replace(vehicle_types=[fleet])
        res = solve(trial, stop=MultipleCriteria([fstop, FirstFeasible()]), seed=seed, collect_stats=False,
                    display=False)
        stats["fleet_solves"] += 1
        if not res.best.is_feasible():
            del trial
            break
        best = res.best
        pr.offer(best)
        feas = fleet.replace(num_available=min(fleet.num_available, best.num_routes()))
        cur = trial
    if mode == "work":
        stats["fleet_iterations"] = fstop.used
        stats["carry"] = carry(fleet_budget - fstop.used)
        dstop = Budget(dist_budget + stats["carry"], deadline)
        dist_open = lambda: not dstop.exhausted()  # noqa: E731
    else:
        dstop = Until(deadline)
        dist_open = lambda: time.monotonic() < deadline  # noqa: E731
    if best is None and not TERM["flag"]:  # already at K, or the first attempt failed: one solve, full fleet
        res = solve(cur, stop=MultipleCriteria([dstop, FirstFeasible()]), seed=seed, collect_stats=False,
                    display=False)
        stats["fleet_solves"] += 1
        if res.best.is_feasible():
            best = res.best
            pr.offer(best)
            feas = feas.replace(num_available=best.num_routes())
    pr.flush(True)
    t_dist = time.monotonic()
    stats["fleet_seconds"] = round(t_dist - t_fleet, 1)
    stats["fleet_vehicles"] = None if best is None else int(best.num_routes())

    # Distance phase: solve with the fleet found, from the fleet phase's solution.
    if best is not None and not TERM["flag"] and dist_open():
        routes0 = [[c - 1 for c in r] for r in solution_routes(best)]
        d2 = cur.replace(vehicle_types=[feas])
        del cur

        class OnBest(IteratedLocalSearchCallbacks):
            def on_best(self, sol):
                pr.offer(sol)

        params = SolveParams(ils=IteratedLocalSearchParams(callbacks=OnBest()))
        res = solve(d2, stop=dstop, seed=seed, collect_stats=False, display=False, params=params,
                    initial_solution=Solution(d2, routes0))
        stats["iterations"] = int(res.num_iterations)
        pr.offer(res.best)
    pr.flush(True)
    stats["distance_seconds"] = round(time.monotonic() - t_dist, 1)
    if mode == "work":
        stats["distance_iterations"] = dstop.used
        stats["clock_stop"] = "fleet" if fstop.clock_stop else ("distance" if dstop.clock_stop else False)
        stats["term_stop"] = bool(fstop.term_stop or dstop.term_stop)
        stats["deterministic"] = not (stats["clock_stop"] or stats["term_stop"] or b["timed_out"])
    else:
        stats["deterministic"] = False
    stats.update({"printed": pr.prints, "vehicles": None if pr.key is None else pr.key[0],
                  "cost": None if pr.key is None else pr.key[1], "sigterm": TERM["flag"],
                  "elapsed": round(time.monotonic() - START, 2)})
    sys.stderr.write("XLTW_SOLVER_STATS %s\n" % json.dumps(stats))
    sys.stderr.flush()
    return stats


def main(argv):
    if len(argv) != 4:
        sys.stderr.write("usage: solver INSTANCE SECONDS SEED\n")
        return 2
    try:
        limit, seed = float(argv[2]), int(argv[3])
        mode = mode_from_env()
    except ValueError as e:
        sys.stderr.write("solver: SECONDS must be a number, SEED an integer and XLTW_MODE work or clock (%s)\n" % e)
        return 2
    if not limit > 0:
        sys.stderr.write("solver: SECONDS must be positive\n")
        return 2
    signal.signal(signal.SIGTERM, _on_term)
    out = claim_stdout()
    run(argv[1], limit, seed, out=out, mode=mode)
    out.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
