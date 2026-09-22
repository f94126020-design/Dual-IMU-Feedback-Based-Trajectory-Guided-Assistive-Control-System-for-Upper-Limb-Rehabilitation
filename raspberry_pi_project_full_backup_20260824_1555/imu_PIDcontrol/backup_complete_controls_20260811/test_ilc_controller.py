import unittest

import numpy as np

from ilc_controller import IterativeLearningController


class ILCControllerTests(unittest.TestCase):
    def _feed_cycle(self, controller, cycle, error=10.0, valid=True):
        period = 4.0
        for index in range(controller.bins):
            t = cycle * period + period * index / controller.bins
            controller.update(t, period, error, valid=valid,
                              invalid_reason="saturated" if not valid else "")

    def test_valid_cycle_updates_bounded_learning(self):
        controller = IterativeLearningController(
            bins=100, learning_gain=0.5, update_limit=2, learned_limit=5,
            q_filter_window=5,
        )
        self._feed_cycle(controller, 0, error=10)
        controller.update(4.0, 4.0, 10)
        self.assertEqual(controller.valid_cycles, 1)
        self.assertLessEqual(np.max(np.abs(controller.learned)), 2.0)
        for cycle in range(1, 5):
            self._feed_cycle(controller, cycle, error=10)
        controller.update(20.0, 4.0, 10)
        self.assertLessEqual(np.max(np.abs(controller.learned)), 5.0)

    def test_invalid_or_incomplete_cycle_does_not_learn(self):
        controller = IterativeLearningController(bins=100)
        for index in range(20):
            controller.update(index * 0.01, 4.0, 10, valid=False, invalid_reason="saturated")
        controller.update(4.0, 4.0, 10)
        self.assertEqual(controller.invalid_cycles, 1)
        self.assertTrue(np.allclose(controller.learned, 0))

    def test_freeze_preserves_curve_but_still_scores_cycle(self):
        controller = IterativeLearningController(bins=100, learning_gain=0.1)
        self._feed_cycle(controller, 0, error=10)
        controller.update(4.0, 4.0, 10)
        learned = controller.learned.copy()
        controller.set_frozen(True)
        self._feed_cycle(controller, 1, error=5)
        controller.update(8.0, 4.0, 5)
        self.assertTrue(np.allclose(controller.learned, learned))
        self.assertEqual(controller.valid_cycles, 2)

    def test_clear_learning_resets_all_results(self):
        controller = IterativeLearningController(bins=100)
        self._feed_cycle(controller, 0)
        controller.update(4.0, 4.0, 10)
        controller.clear_learning()
        self.assertEqual(controller.valid_cycles, 0)
        self.assertTrue(np.allclose(controller.learned, 0))


if __name__ == "__main__":
    unittest.main()
