"""Tests for xltw-solver 0.2.0's work budget (deterministic stopping). Standard library only; PyVRP is not needed.

MV2 of Munim, Inc. (mv2@munimtech.com). Made with AI agents working in MV2's name. Apache-2.0.
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import solver  # noqa: E402

# Slowest per-phase rates measured in cvrplib-baselines xltwdet/ (2026-11-07), XLTW-Test-n1234 under a CPU hog.
SLOW_FLEET_N1234 = 116.0
SLOW_DIST_N1234 = 250.0


class Budgets(unittest.TestCase):
    def test_version_and_settings(self):
        self.assertEqual(solver.VERSION, "0.2.0")
        self.assertEqual(solver.FLEET_FRAC, 0.8)
        self.assertLess(solver.RF0, solver.RD0)

    def test_machine_free_and_repeatable(self):
        self.assertEqual(solver.phase_budgets(1233, 2219.4), solver.phase_budgets(1233, 2219.4))
        self.assertEqual(solver.phase_budgets(1233, "2219.4"), solver.phase_budgets(1233, 2219.4))

    def test_shape(self):
        f1, d1 = solver.phase_budgets(1000, 1800.0)
        usable = 1800.0 - solver.margin(1800.0)
        self.assertEqual(f1, int(solver.RF0 * solver.FLEET_FRAC * usable))
        self.assertEqual(d1, int(solver.RD0 * (1 - solver.FLEET_FRAC) * usable))
        f2, d2 = solver.phase_budgets(4000, 1800.0)              # 4x the customers: half the rate
        self.assertAlmostEqual(f2 / f1, 0.5, places=3)
        self.assertAlmostEqual(d2 / d1, 0.5, places=3)
        f3, d3 = solver.phase_budgets(1000, 3600.0)
        self.assertGreater(f3, 1.99 * f1)
        self.assertGreater(d3, 1.99 * d1)
        self.assertEqual(solver.phase_budgets(1000, 1.0), (0, 0))  # all of a tiny limit is margin

    def test_fits_the_slowest_measured_machine(self):
        """At n1234's speed under a CPU hog the budgets end well before the limit, at 120 s and at T(n)."""
        for limit in (120.0, 2219.4):
            f, d = solver.phase_budgets(1233, limit)
            need = f / SLOW_FLEET_N1234 + d / SLOW_DIST_N1234
            self.assertLess(need, 0.85 * (limit - solver.margin(limit)), limit)
            self.assertGreater(need, 0.40 * limit, limit)  # and does not throw most of the limit away

    def test_carry(self):
        self.assertEqual(solver.carry(0), 0)
        self.assertEqual(solver.carry(-5), 0)
        self.assertEqual(solver.carry(9), int(9 * solver.RD0 / solver.RF0))
        self.assertGreaterEqual(solver.carry(1000), 1000)

    def test_size_factor(self):
        self.assertAlmostEqual(solver.size_factor(1000), 1.0)
        self.assertAlmostEqual(solver.size_factor(250), 2.0)
        self.assertAlmostEqual(solver.size_factor(0), solver.size_factor(1))


class BudgetCriterion(unittest.TestCase):
    def tearDown(self):
        solver.TERM["flag"] = False

    def test_counts_exactly(self):
        now = [0.0]
        b = solver.Budget(3, deadline=100.0, clock=lambda: now[0])
        self.assertEqual([b(0.0) for _ in range(5)], [False, False, False, True, True])
        self.assertEqual((b.used, b.left), (3, 0))
        self.assertTrue(b.exhausted())
        self.assertFalse(b.clock_stop or b.term_stop)

    def test_clock_is_only_a_safety_net(self):
        now = [0.0]
        b = solver.Budget(10, deadline=5.0, clock=lambda: now[0])
        self.assertFalse(b(0.0))
        now[0] = 5.0
        self.assertTrue(b(0.0))
        self.assertTrue(b.clock_stop)
        self.assertTrue(b.exhausted())
        self.assertEqual(b.used, 1)

    def test_sigterm(self):
        b = solver.Budget(10, deadline=1e18, clock=lambda: 0.0)
        solver.TERM["flag"] = True
        self.assertTrue(b(0.0))
        self.assertTrue(b.term_stop)


class Mode(unittest.TestCase):
    def test_default_is_work(self):
        self.assertEqual(solver.mode_from_env({}), "work")
        self.assertEqual(solver.mode_from_env({"XLTW_MODE": ""}), "work")
        self.assertEqual(solver.mode_from_env({"XLTW_MODE": "clock"}), "clock")
        with self.assertRaises(ValueError):
            solver.mode_from_env({"XLTW_MODE": "fast"})

    def test_no_crlf(self):
        for name in ("solver.py", "test_budget.py"):
            with open(os.path.join(HERE, name), "rb") as f:
                self.assertNotIn(b"\r", f.read(), name)


if __name__ == "__main__":
    unittest.main()
