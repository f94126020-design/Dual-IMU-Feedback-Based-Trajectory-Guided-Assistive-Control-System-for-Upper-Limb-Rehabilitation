import types
import unittest

import web_GUI_control as web


class WebSafetyTests(unittest.TestCase):
    def setUp(self):
        web.rom_calibration.reset()
        web.sensor = types.SimpleNamespace(motor_enabled=False, emergency_stop=False)
        web.initialized = True
        web.initializing = False
        web.motor_bridge = None
        web.motor_bridge_connected = False
        web.muscle_allocator.reset_conditioner()
        self.client = web.app.test_client()

    def tearDown(self):
        web.sensor = None
        web.initialized = False
        web.rom_calibration.reset()

    def test_home_and_status_are_available(self):
        home = self.client.get("/")
        self.assertEqual(home.status_code, 200)
        html = home.get_data(as_text=True)
        self.assertIn('id="validationStageButton"', html)
        self.assertIn('id="validationApplyButton"', html)
        self.assertIn("病患個人化活動範圍 ROM", html)
        self.assertIn("方案 B DRY RUN", html)
        self.assertIn("肘 90°－三頭肌伸展至打直", html)
        self.assertIn("不要把手臂舉過頭", html)
        self.assertIn('id="validationProtocol"', html)
        self.assertIn('id="validationLiveAngles"', html)
        self.assertIn('id="validationProgressBar"', html)
        response = self.client.get("/api/status")
        self.assertEqual(response.status_code, 200)
        status = response.get_json()
        self.assertIn("rom_calibration", status)
        self.assertNotIn("validation", status)
        self.assertEqual(status["rom_calibration"]["mode"], "patient_rom")
        self.assertEqual(status["motor_output"]["control_profile"], "three_muscle")
        self.assertEqual(status["motor_output"]["pwm_limit"], 255)
        self.assertEqual(status["motor_output"]["antagonist_release_gain"], 0.8)
        self.assertEqual(status["motor_output"]["command_filter_tau"], 0.18)
        self.assertEqual(status["motor_output"]["reverse_deadtime"], 0.15)
        self.assertEqual(status["motor_output"]["shoulder_release_max_pwm"], 60.0)
        self.assertFalse(status["motor_output"]["live_output_allowed"])

    def test_motor_enable_is_locked_by_dry_run(self):
        response = self.client.post("/api/set_motor", json={"enabled": True})
        self.assertEqual(response.status_code, 403)
        self.assertFalse(response.get_json()["ok"])
        self.assertFalse(web.sensor.motor_enabled)

    def test_motor_enable_remains_locked_after_rom_in_dry_run(self):
        web.rom_calibration.applied = True
        response = self.client.post("/api/set_motor", json={"enabled": True})
        self.assertEqual(response.status_code, 403)
        self.assertIn("DRY RUN", response.get_json()["message"])
        self.assertFalse(web.sensor.motor_enabled)

    def test_rom_start_keeps_motor_disabled(self):
        web.sensor.motor_enabled = True
        response = self.client.post("/api/rom/start", json={})
        self.assertEqual(response.status_code, 200)
        self.assertFalse(web.sensor.motor_enabled)
        self.assertEqual(response.get_json()["rom_calibration"]["expected_stage"], "elbow_flexion")

    def test_old_validation_api_is_removed(self):
        response = self.client.post("/api/validation/start", json={})
        self.assertEqual(response.status_code, 404)

    def test_server_accepts_shoulder_and_removes_30_pwm_limit(self):
        response = self.client.post("/api/apply_params", json={
            "target_joint": "shoulder",
            "target_mode": "fixed",
            "fixed_target_angle": 55,
            "output_limit": 999,
        })
        self.assertEqual(response.status_code, 200)
        self.assertEqual(web.sensor.target_joint, "shoulder")
        self.assertEqual(web.sensor.target_mode, "fixed")
        self.assertEqual(web.sensor.fixed_target_angle, 55)
        self.assertEqual(web.sensor.output_limit, 255)


if __name__ == "__main__":
    unittest.main()
