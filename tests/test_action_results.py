import unittest
from robowrap.helperFuncs import wait_for_action

class ActionResults(unittest.TestCase):
    def test_completed_failure_is_not_success(self):
        class Action:
            has_failed = True
            def wait_for_completed(self, timeout=None):
                return True
        action = Action()
        self.assertFalse(wait_for_action(action))
        self.assertIs(wait_for_action(action, blocking=False), action)
        action.has_failed = False
        self.assertTrue(wait_for_action(action, timeout=1))
