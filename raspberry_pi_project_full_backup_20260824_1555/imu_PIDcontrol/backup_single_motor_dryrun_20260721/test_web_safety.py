import types
import unittest

import web_GUI_control as web


class WebSafetyTests(unittest.TestCase):
    def setUp(self):
        web.imu_validation.reset()
        web.sensor = types.SimpleNamespace(motor_enabled=False, emergency_stop=False)
        web.initialized = True
        web.initializing = False
        web.motor_bridge = None
        web.motor_bridge_connected = False
        self.client = web.app.test_client()

    def tearDown(self):
        web.sensor = None
        web.initialized = False
        web.imu_validation.reset()

    def test_home_and_status_are_available(self):
        home = self.client.get("/")
        self.assertEqual(home.status_code, 200)
        html = home.get_data(as_text=True)
        self.assertIn('id="validationStageButton"', html)
        self.assertIn('id="validationApplyButton"', html)
        self.assertIn("不代表絕對角度準確度", html)
        response = self.client.get("/api/status")
        self.assertEqual(response.status_code, 200)
        self.assertIn("validation", response.get_json())

    def test_motor_enable_is_locked_before_validation(self):
        response = self.client.post("/api/set_motor", json={"enabled": True})
        self.assertEqual(response.status_code, 403)
        self.assertFalse(response.get_json()["ok"])
        self.assertFalse(web.sensor.motor_enabled)

    def test_validation_start_keeps_motor_disabled(self):
        web.sensor.motor_enabled = True
        response = self.client.post("/api/validation/start", json={})
        self.assertEqual(response.status_code, 200)
        self.assertFalse(web.sensor.motor_enabled)
        self.assertEqual(response.get_json()["validation"]["expected_stage"], "static")


if __name__ == "__main__":
    unittest.main()
