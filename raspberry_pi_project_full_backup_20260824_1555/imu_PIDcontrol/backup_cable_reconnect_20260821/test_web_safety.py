import os
import tempfile
import types
import unittest
from unittest import mock

import web_GUI_control as web


class FakeSensor(types.SimpleNamespace):
    def apply_rom_calibration(self, **mapping):
        self.applied_rom_mapping = mapping


class WebSafetyTests(unittest.TestCase):
    def setUp(self):
        web.rom_calibration.reset()
        web.frequency_session.reset()
        web.sensor = FakeSensor(motor_enabled=False, emergency_stop=False)
        web.initialized = True
        web.initializing = False
        web.recording = False
        web.csv_file = None
        web.csv_writer = None
        web.motor_bridge = None
        web.motor_bridge_connected = False
        web.muscle_allocator.reset_conditioner()
        self.client = web.app.test_client()

    def tearDown(self):
        web.sensor = None
        web.initialized = False
        web.initializing = False
        if web.recording:
            self.client.post("/api/stop_recording", json={})
        web.rom_calibration.reset()
        web.frequency_session.reset()

    def test_home_and_status_are_available(self):
        home = self.client.get("/")
        self.assertEqual(home.status_code, 200)
        html = home.get_data(as_text=True)
        self.assertIn('id="validationStageButton"', html)
        self.assertIn('id="validationApplyButton"', html)
        self.assertIn("病患個人化活動範圍 ROM", html)
        self.assertIn('id="romDurationInput"', html)
        self.assertIn("自由重複", html)
        self.assertIn("持續柔和助力 LIVE", html)
        self.assertIn("window.confirm", html)
        self.assertIn("持續柔和助力（前饋＋PID）", html)
        self.assertIn("開迴路軌跡展示（僅測試）", html)
        self.assertIn("LADRC 自抗擾控制", html)
        self.assertIn("前饋＋LADRC", html)
        self.assertIn("ILC＋PID（週期學習）", html)
        self.assertIn("ILC＋LADRC（週期學習）", html)
        self.assertIn('id="trajectoryFixedPanel"', html)
        self.assertIn('id="trajectorySinePanel"', html)
        self.assertIn('id="trajectoryStepPanel"', html)
        self.assertIn('id="pidParameterPanel"', html)
        self.assertIn('id="feedforwardParameterPanel"', html)
        self.assertIn('id="adrcParameterPanel"', html)
        self.assertIn('id="ilcParameterPanel"', html)
        self.assertIn('id="demoParameterPanel"', html)
        self.assertIn("showOnlyPanels('.controller-panel'", html)
        self.assertIn("showOnlyPanels('.trajectory-panel'", html)
        self.assertIn('id="actionToast"', html)
        self.assertIn("button.button-busy", html)
        self.assertIn("finishButtonAction", html)
        self.assertIn("document.addEventListener('click'", html)
        self.assertIn('id="measuredAngle"', html)
        self.assertIn('id="targetAngle"', html)
        self.assertIn('id="errorValue"', html)
        self.assertIn('id="elbowAngle"', html)
        self.assertIn('id="shoulderAngle"', html)
        self.assertIn('id="motionState"', html)
        self.assertNotIn('id="desiredPwmBiceps"', html)
        self.assertNotIn('id="desiredPwmTriceps"', html)
        self.assertNotIn('id="desiredPwmDeltoid"', html)
        self.assertNotIn('id="pwmBiceps"', html)
        self.assertNotIn('id="pwmTriceps"', html)
        self.assertNotIn('id="pwmDeltoid"', html)
        self.assertNotIn('id="pidOutput"', html)
        self.assertNotIn('id="adrcDisturbance"', html)
        self.assertNotIn('id="ilcOutput"', html)
        self.assertNotIn('id="serialDebugText"', html)
        self.assertIn("禁止連接病患", html)
        self.assertIn("系統識別與頻域分析", html)
        self.assertIn("Chirp 掃頻", html)
        self.assertIn("PRBS", html)
        self.assertNotIn('<option value="p">', html)
        self.assertNotIn('<option value="pi">', html)
        self.assertNotIn("Servo Cmd", html)
        self.assertIn("肘 90°－三頭肌伸展至打直", html)
        self.assertIn("不要把上臂舉過頭", html)
        self.assertIn('id="validationProtocol"', html)
        self.assertIn('id="validationLiveAngles"', html)
        self.assertIn('id="validationProgressBar"', html)
        self.assertIn("按鍵已收到，正在確認目前 ROM 階段", html)
        self.assertIn("statusMessageUntil", html)
        self.assertIn("setLiveStatus(data.status)", html)
        self.assertIn("setTimeout(pollStatus,250)", html)
        self.assertNotIn("setInterval(", html)
        response = self.client.get("/api/status")
        self.assertEqual(response.status_code, 200)
        status = response.get_json()
        self.assertIn("rom_calibration", status)
        self.assertIn("imu_health", status)
        self.assertIn("frequency_identification", status)
        self.assertNotIn("validation", status)
        self.assertEqual(status["rom_calibration"]["mode"], "patient_rom")
        self.assertEqual(status["motor_output"]["control_profile"], "three_muscle")
        self.assertEqual(status["motor_output"]["pwm_limit"], 255)
        self.assertEqual(status["motor_output"]["antagonist_release_gain"], 0.8)
        self.assertEqual(status["motor_output"]["command_filter_tau"], 0.08)
        self.assertEqual(status["motor_output"]["reverse_deadtime"], 0.08)
        self.assertTrue(status["motor_output"]["continuous_assist"])
        self.assertEqual(status["motor_output"]["feedforward_gain"], 0.35)
        self.assertEqual(status["motor_output"]["feedforward_min_pwm"], 8.0)
        self.assertTrue(status["motor_output"]["trajectory_demo"])
        self.assertTrue(status["motor_output"]["adrc"])
        self.assertTrue(status["motor_output"]["ilc"])
        self.assertIn("ilc", status)
        self.assertEqual(status["motor_output"]["demo_output_limit"], 15.0)
        self.assertEqual(status["motor_output"]["shoulder_release_max_pwm"], 60.0)
        self.assertTrue(status["motor_output"]["live_output_allowed"])
        self.assertEqual(status["motor_output"]["mode"], "live_on_confirmation")

    def test_motor_enable_requires_explicit_confirmation(self):
        response = self.client.post("/api/set_motor", json={"enabled": True})
        self.assertEqual(response.status_code, 403)
        self.assertFalse(response.get_json()["ok"])
        self.assertFalse(web.sensor.motor_enabled)

    def test_confirmed_motor_enable_arms_esp32_without_rom_gate(self):
        bridge = mock.Mock()
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True):
            response = self.client.post(
                "/api/set_motor", json={"enabled": True, "confirmed": True}
            )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["ok"])
        self.assertTrue(web.sensor.motor_enabled)
        bridge.stop.assert_called_once()
        bridge.arm.assert_called_once()

    def test_arm_failure_keeps_motor_stopped(self):
        bridge = mock.Mock()
        bridge.arm.side_effect = RuntimeError("not armed")
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True):
            response = self.client.post(
                "/api/set_motor", json={"enabled": True, "confirmed": True}
            )
        self.assertEqual(response.status_code, 503)
        self.assertFalse(response.get_json()["ok"])
        self.assertFalse(web.sensor.motor_enabled)

    def test_rom_start_keeps_motor_disabled(self):
        web.sensor.motor_enabled = True
        response = self.client.post("/api/rom/start", json={"duration_seconds": 12})
        self.assertEqual(response.status_code, 200)
        self.assertFalse(web.sensor.motor_enabled)
        self.assertEqual(response.get_json()["rom_calibration"]["expected_stage"], "elbow_flexion")
        self.assertEqual(response.get_json()["rom_calibration"]["stage_duration_seconds"], 12.0)

    def test_rom_stage_endpoint_starts_once_and_rejects_duplicate(self):
        self.client.post("/api/rom/start", json={})
        response = self.client.post("/api/rom/start_stage", json={})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["rom_calibration"]["current_stage"], "elbow_flexion")
        duplicate = self.client.post("/api/rom/start_stage", json={})
        self.assertEqual(duplicate.status_code, 400)
        self.assertFalse(duplicate.get_json()["ok"])

    def test_rom_apply_endpoint_updates_sensor_targets(self):
        web.rom_calibration.results = {stage: {"recorded": True} for stage in web.rom_calibration.STAGE_ORDER}
        web.rom_calibration.mapping = {
            "elbow_axis": "roll",
            "elbow_sign": 1,
            "front_reference": (1.0, 0.0, 0.0),
            "side_reference": (0.0, 0.0, 1.0),
        }
        web.rom_calibration.rom = {
            "elbow_extension_target_deg": 5.0,
            "elbow_flexion_target_deg": 95.0,
            "shoulder_front_target_deg": 90.0,
            "shoulder_side_target_deg": 80.0,
        }
        response = self.client.post("/api/rom/apply", json={})
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["ok"])
        self.assertEqual(web.sensor.trajectory_min_angle, 5.0)
        self.assertEqual(web.sensor.trajectory_max_angle, 95.0)
        self.assertEqual(web.sensor.fixed_target_angle, 95.0)

    def test_disable_emergency_and_clear_buttons(self):
        web.sensor.motor_enabled = True
        response = self.client.post("/api/set_motor", json={"enabled": False})
        self.assertTrue(response.get_json()["ok"])
        self.assertFalse(web.sensor.motor_enabled)
        response = self.client.post("/api/emergency_stop", json={})
        self.assertTrue(response.get_json()["ok"])
        self.assertTrue(web.sensor.emergency_stop)
        self.assertFalse(web.sensor.motor_enabled)
        response = self.client.post("/api/clear_emergency", json={})
        self.assertTrue(response.get_json()["ok"])
        self.assertFalse(web.sensor.emergency_stop)

    def test_recording_buttons_create_and_close_csv(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            response = self.client.post("/api/start_recording", json={
                "participant_id": "BUTTON",
                "session_name": "test",
                "label": "controls",
                "output_dir": temp_dir,
            })
            payload = response.get_json()
            self.assertTrue(payload["ok"])
            self.assertTrue(os.path.isfile(payload["csv_path"]))
            duplicate = self.client.post("/api/start_recording", json={"output_dir": temp_dir})
            self.assertFalse(duplicate.get_json()["ok"])
            stopped = self.client.post("/api/stop_recording", json={})
            self.assertTrue(stopped.get_json()["ok"])
            self.assertFalse(web.recording)

    def test_initialize_button_dispatches_worker_without_running_hardware(self):
        with mock.patch.object(web.threading, "Thread") as thread_class:
            response = self.client.post("/api/init", json={})
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["ok"])
        self.assertTrue(web.initializing)
        thread_class.return_value.start.assert_called_once()

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

    def test_demo_rejects_non_sine_trajectory(self):
        response = self.client.post("/api/apply_params", json={
            "controller_mode": "trajectory_demo",
            "target_mode": "step",
        })
        self.assertEqual(response.status_code, 400)
        self.assertFalse(response.get_json()["ok"])
        init_response = self.client.post("/api/init", json={
            "controller_mode": "trajectory_demo",
            "target_mode": "fixed",
        })
        self.assertEqual(init_response.status_code, 400)

    def test_adrc_rejects_zero_input_gain(self):
        response = self.client.post("/api/apply_params", json={
            "controller_mode": "adrc",
            "target_mode": "sine",
            "adrc_input_gain": 0,
        })
        self.assertEqual(response.status_code, 400)

    def test_ilc_rejects_non_sine_and_reset_requires_disabled_motor(self):
        response = self.client.post("/api/apply_params", json={
            "controller_mode": "ilc_pid",
            "target_mode": "step",
        })
        self.assertEqual(response.status_code, 400)
        ilc = mock.Mock()
        ilc.status.return_value = {"enabled": True}
        web.sensor.ilc = ilc
        web.sensor.motor_enabled = True
        blocked = self.client.post("/api/ilc/reset", json={})
        self.assertEqual(blocked.status_code, 409)
        web.sensor.motor_enabled = False
        reset = self.client.post("/api/ilc/reset", json={})
        self.assertEqual(reset.status_code, 200)
        ilc.clear_learning.assert_called_once()

    def test_demo_enable_resets_trajectory_after_arm(self):
        bridge = mock.Mock()
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        web.sensor.controller_mode = "trajectory_demo"
        web.sensor.target_mode = "sine"
        web.sensor.reset_trajectory = mock.Mock()
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True):
            response = self.client.post(
                "/api/set_motor", json={"enabled": True, "confirmed": True}
            )
        self.assertEqual(response.status_code, 200)
        web.sensor.reset_trajectory.assert_called_once()

    def test_frequency_start_requires_rig_confirmation(self):
        response = self.client.post("/api/frequency/start", json={})
        self.assertEqual(response.status_code, 403)
        self.assertFalse(response.get_json()["ok"])

    def test_frequency_start_arms_and_stop_disables_motor(self):
        bridge = mock.Mock()
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        web.sensor.target_joint = "elbow"
        web.latest_frame = {"elbow_angle": 10.0, "shoulder_angle": 0.0}
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True):
            response = self.client.post("/api/frequency/start", json={
                "confirmed_rig": True,
                "target_joint": "elbow",
                "signal_type": "chirp",
                "duration": 10,
                "amplitude": 8,
                "f_start": 0.1,
                "f_end": 2.0,
                "safe_min_angle": -10,
                "safe_max_angle": 120,
            })
        self.assertEqual(response.status_code, 200)
        self.assertTrue(web.sensor.motor_enabled)
        self.assertEqual(web.frequency_session.state, "running")
        bridge.stop.assert_called_once()
        bridge.arm.assert_called_once()
        stopped = self.client.post("/api/frequency/stop", json={})
        self.assertEqual(stopped.status_code, 200)
        self.assertFalse(web.sensor.motor_enabled)
        self.assertEqual(web.frequency_session.state, "aborted")


if __name__ == "__main__":
    unittest.main()
