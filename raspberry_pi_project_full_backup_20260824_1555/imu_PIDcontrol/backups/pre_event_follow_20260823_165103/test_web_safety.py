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
        web.motor_test_running = False
        web.motor_jog_active = False
        web.motor_jog_session += 1
        web.motor_jog_last_heartbeat = 0.0
        web.motor_channel_locks.update({
            "biceps": False, "triceps": False, "deltoid": False,
        })
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
        web.motor_test_running = False
        web.motor_jog_active = False
        web.motor_jog_session += 1

    def test_home_and_status_are_available(self):
        home = self.client.get("/")
        self.assertEqual(home.status_code, 200)
        html = home.get_data(as_text=True)
        self.assertIn('id="validationStageButton"', html)
        self.assertIn('id="validationApplyButton"', html)
        self.assertIn("病患個人化活動範圍 ROM", html)
        self.assertIn('id="romDurationInput"', html)
        self.assertIn("自由重複", html)
        self.assertIn("IMU 動作跟隨 LIVE", html)
        self.assertIn("window.confirm", html)
        self.assertIn("PID（IMU 跟隨＋落後補償）", html)
        self.assertIn("開迴路軌跡展示（僅測試）", html)
        self.assertIn("LADRC 自抗擾控制", html)
        self.assertIn("純 IMU 跟隨（無目標、無補償）", html)
        self.assertNotIn('<option value="assist">', html)
        self.assertNotIn('<option value="continuous_assist">', html)
        self.assertNotIn('<option value="feedforward_adrc">', html)
        self.assertIn("ILC＋PID（週期學習）", html)
        self.assertIn("ILC＋LADRC（週期學習）", html)
        self.assertIn('id="trajectoryFixedPanel"', html)
        self.assertIn('id="trajectorySinePanel"', html)
        self.assertIn('id="trajectoryStepPanel"', html)
        self.assertIn('id="pidParameterPanel"', html)
        self.assertIn('id="kpInput" value="5"', html)
        self.assertIn('id="feedforwardParameterPanel"', html)
        self.assertIn('id="feedforwardMinInput" value="10"', html)
        self.assertNotIn('id="feedforwardGainInput"', html)
        self.assertIn("設定多少就輸出多少", html)
        self.assertIn('id="adrcParameterPanel"', html)
        self.assertIn('id="ilcParameterPanel"', html)
        self.assertIn('id="demoParameterPanel"', html)
        self.assertIn('id="demoOutputLimitInput" value="100"', html)
        self.assertIn("showOnlyPanels('.controller-panel'", html)
        self.assertIn("showOnlyPanels('.trajectory-panel'", html)
        self.assertIn('id="actionToast"', html)
        self.assertIn("button.button-busy", html)
        self.assertIn("finishButtonAction", html)
        self.assertIn("document.addEventListener('click'", html)
        self.assertIn('id="motorJogPwm"', html)
        self.assertIn('id="motorJogConfirmed"', html)
        self.assertIn('value="100"', html)
        self.assertIn("startMotorJog(event,'biceps',1)", html)
        self.assertIn("startMotorJog(event,'triceps',-1)", html)
        self.assertIn("startMotorJog(event,'deltoid',1)", html)
        self.assertIn("/api/motor_jog/heartbeat", html)
        self.assertIn('id="motorJogLockBiceps"', html)
        self.assertIn('id="motorJogLockTriceps"', html)
        self.assertIn('id="motorJogLockDeltoid"', html)
        self.assertIn("/api/motor_locks", html)
        self.assertIn("所有復健／測試模式都固定輸出 0", html)
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
        self.assertIn("const livePollDelayMs = 40", html)
        self.assertIn("const slowUiIntervalMs = 500", html)
        self.assertIn("setTimeout(pollStatus,document.hidden?1000:livePollDelayMs)", html)
        self.assertIn("cache:'no-store'", html)
        self.assertIn("const maxPoints = 160", html)
        self.assertIn("requestAnimationFrame", html)
        self.assertNotIn("setInterval(pollStatus", html)
        self.assertIn("setInterval(motorJogHeartbeat,150)", html)
        self.assertIn("fixed-estop", html)
        self.assertIn("parameterStorageKey", html)
        response = self.client.get("/api/status")
        self.assertEqual(response.status_code, 200)
        status = response.get_json()
        self.assertIn("rom_calibration", status)
        self.assertIn("imu_health", status)
        self.assertIn("reconnect_count", status["imu_health"])
        self.assertIn("transient_retry_count", status["imu_health"])
        self.assertIn("reconnecting", status["imu_health"])
        self.assertEqual(status["imu_health"]["hard_stall_timeout_seconds"], 15.0)
        self.assertIn("frequency_identification", status)
        self.assertNotIn("validation", status)
        self.assertEqual(status["rom_calibration"]["mode"], "patient_rom")
        self.assertEqual(status["motor_output"]["control_profile"], "three_muscle")
        self.assertEqual(status["motor_output"]["pwm_limit"], 255)
        self.assertEqual(status["motor_output"]["motor_min_pwm"], {"biceps": 0, "triceps": 0, "deltoid": 0})
        self.assertEqual(status["motor_test"]["pwm"], 200)
        self.assertEqual(status["motor_test"]["duration_seconds"], 0.5)
        self.assertFalse(status["motor_jog"]["active"])
        self.assertEqual(status["motor_jog"]["default_pwm"], 100)
        self.assertEqual(status["motor_jog"]["max_pwm"], 200)
        self.assertEqual(status["motor_jog"]["heartbeat_timeout_seconds"], 0.5)
        self.assertEqual(status["motor_jog"]["locks"], {
            "biceps": False, "triceps": False, "deltoid": False,
        })
        self.assertEqual(status["motor_locks"], {
            "biceps": False, "triceps": False, "deltoid": False,
        })
        self.assertEqual(status["motor_output"]["antagonist_release_gain"], 1.0)
        self.assertEqual(status["motor_output"]["triceps_release_ratio"], 0.5)
        self.assertEqual(status["motor_output"]["biceps_release_ratio"], 0.8)
        self.assertEqual(status["motor_output"]["cable_return_gain"], 1.0)
        self.assertIn("virtual_cable_effort", status["motor_output"])
        self.assertEqual(status["motor_output"]["command_filter_tau"], 0.02)
        self.assertEqual(status["motor_output"]["wind_slew_rate"], 1200.0)
        self.assertEqual(status["motor_output"]["release_slew_rate"], 2000.0)
        self.assertEqual(status["motor_output"]["reverse_deadtime"], 0.02)
        self.assertTrue(status["motor_output"]["imu_following"])
        self.assertEqual(status["motor_output"]["controller_modes"], ["pid", "adrc", "ilc_pid", "ilc_adrc", "following_only", "trajectory_demo"])
        self.assertEqual(status["motor_output"]["feedforward_gain"], 0.0)
        self.assertEqual(status["motor_output"]["feedforward_min_pwm"], 10.0)
        self.assertTrue(status["motor_output"]["trajectory_demo"])
        self.assertTrue(status["motor_output"]["adrc"])
        self.assertTrue(status["motor_output"]["ilc"])
        self.assertIn("ilc", status)
        self.assertEqual(status["motor_output"]["demo_output_limit"], 100.0)
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

    def test_motor_test_requires_confirmation_and_disabled_control(self):
        denied = self.client.post("/api/motor_test/pulse", json={
            "motor": "triceps", "direction": 1,
        })
        self.assertEqual(denied.status_code, 403)
        web.sensor.motor_enabled = True
        blocked = self.client.post("/api/motor_test/pulse", json={
            "motor": "triceps", "direction": 1, "confirmed_rig": True,
        })
        self.assertEqual(blocked.status_code, 409)

    def test_motor_test_arms_then_dispatches_bounded_worker(self):
        bridge = mock.Mock()
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True), mock.patch.object(web.threading, "Thread") as thread_class:
            response = self.client.post("/api/motor_test/pulse", json={
                "motor": "deltoid", "direction": -1, "confirmed_rig": True,
            })
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["ok"])
        bridge.stop.assert_called_once()
        bridge.arm.assert_called_once()
        thread_class.return_value.start.assert_called_once()

    def test_motor_jog_requires_confirmation_and_disabled_main_control(self):
        denied = self.client.post("/api/motor_jog/start", json={
            "motor": "biceps", "direction": 1, "pwm": 100,
        })
        self.assertEqual(denied.status_code, 403)
        web.sensor.motor_enabled = True
        blocked = self.client.post("/api/motor_jog/start", json={
            "motor": "biceps", "direction": 1, "pwm": 100, "confirmed": True,
        })
        self.assertEqual(blocked.status_code, 409)

    def test_motor_jog_is_available_before_imu_initialization(self):
        bridge = mock.Mock(safety_state="IDLE")
        web.sensor = None
        web.initialized = False
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True), mock.patch.object(web.threading, "Thread") as thread_class:
            response = self.client.post("/api/motor_jog/start", json={
                "motor": "biceps", "direction": 1, "pwm": 100, "confirmed": True,
            })
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["ok"])
        bridge.stop.assert_called_once()
        bridge.arm.assert_called_once()
        thread_class.return_value.start.assert_called_once()

    def test_motor_jog_lock_blocks_only_selected_motor(self):
        response = self.client.post("/api/motor_locks", json={
            "motor": "biceps", "locked": True,
        })
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["locks"]["biceps"])

        blocked = self.client.post("/api/motor_jog/start", json={
            "motor": "biceps", "direction": 1, "pwm": 100, "confirmed": True,
        })
        self.assertEqual(blocked.status_code, 409)
        self.assertFalse(blocked.get_json()["ok"])

        bridge = mock.Mock(safety_state="IDLE")
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True), mock.patch.object(web.threading, "Thread") as thread_class:
            allowed = self.client.post("/api/motor_jog/start", json={
                "motor": "triceps", "direction": 1, "pwm": 100, "confirmed": True,
            })
        self.assertEqual(allowed.status_code, 200)
        self.assertTrue(allowed.get_json()["ok"])
        thread_class.return_value.start.assert_called_once()

    def test_locking_active_jog_stops_motor_immediately(self):
        bridge = mock.Mock()
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        web.motor_test_running = True
        web.motor_jog_active = True
        web.motor_jog_motor = "deltoid"
        previous_session = web.motor_jog_session
        response = self.client.post("/api/motor_locks", json={
            "motor": "deltoid", "locked": True,
        })
        self.assertEqual(response.status_code, 200)
        self.assertFalse(web.motor_jog_active)
        self.assertFalse(web.motor_test_running)
        self.assertGreater(web.motor_jog_session, previous_session)
        bridge.stop.assert_called_once()

    def test_global_motor_lock_is_applied_at_final_esp32_gateway(self):
        web.motor_channel_locks.update({
            "biceps": True, "triceps": False, "deltoid": True,
        })
        locked = web.apply_motor_channel_locks(
            web.MusclePWM(biceps=200, triceps=-100, deltoid=80)
        )
        self.assertEqual(
            locked,
            web.MusclePWM(biceps=0, triceps=-100, deltoid=0),
        )
        bridge = mock.Mock()
        web.motor_bridge = bridge
        sent = web.send_motor_pwm_with_locks(200, -100, 80)
        self.assertEqual(sent, locked)
        bridge.set_pwm.assert_called_once_with(0, -100, 0)

    def test_global_motor_lock_blocks_pulse_test_for_selected_motor(self):
        web.motor_channel_locks["triceps"] = True
        response = self.client.post("/api/motor_test/pulse", json={
            "motor": "triceps", "direction": 1, "confirmed_rig": True,
        })
        self.assertEqual(response.status_code, 409)
        self.assertFalse(response.get_json()["ok"])

    def test_motor_jog_arms_clamps_pwm_heartbeats_and_stops(self):
        bridge = mock.Mock()
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True), mock.patch.object(web.threading, "Thread") as thread_class:
            response = self.client.post("/api/motor_jog/start", json={
                "motor": "triceps", "direction": -1, "pwm": 999, "confirmed": True,
            })
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["pwm"], 200)
        self.assertTrue(web.motor_jog_active)
        self.assertTrue(web.motor_test_running)
        bridge.stop.assert_called_once()
        bridge.arm.assert_called_once()
        thread_class.return_value.start.assert_called_once()

        heartbeat = self.client.post("/api/motor_jog/heartbeat", json={})
        self.assertEqual(heartbeat.status_code, 200)
        stopped = self.client.post("/api/motor_jog/stop", json={})
        self.assertEqual(stopped.status_code, 200)
        self.assertFalse(web.motor_jog_active)
        self.assertFalse(web.motor_test_running)

    def test_motor_jog_worker_stops_on_missing_browser_heartbeat(self):
        bridge = mock.Mock()
        web.motor_bridge = bridge
        web.motor_test_running = True
        web.motor_jog_active = True
        web.motor_jog_session += 1
        session_id = web.motor_jog_session
        web.motor_jog_last_heartbeat = 0.0
        web.motor_jog_worker(session_id, "biceps", 1, 100)
        self.assertFalse(web.motor_jog_active)
        self.assertFalse(web.motor_test_running)
        bridge.set_pwm.assert_not_called()
        bridge.stop.assert_called_once()

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
        bridge = mock.Mock()
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        web.sensor.motor_enabled = True
        response = self.client.post("/api/set_motor", json={"enabled": False})
        self.assertTrue(response.get_json()["ok"])
        self.assertFalse(web.sensor.motor_enabled)
        response = self.client.post("/api/emergency_stop", json={})
        self.assertTrue(response.get_json()["ok"])
        self.assertTrue(web.sensor.emergency_stop)
        self.assertFalse(web.sensor.motor_enabled)
        bridge.emergency_stop.assert_called_once()
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True):
            response = self.client.post("/api/clear_emergency", json={})
        self.assertTrue(response.get_json()["ok"])
        self.assertFalse(web.sensor.emergency_stop)
        bridge.clear_fault.assert_called_once()

    def test_unconfirmed_estop_and_clear_keep_software_latched(self):
        response = self.client.post("/api/emergency_stop", json={})
        self.assertEqual(response.status_code, 503)
        self.assertFalse(response.get_json()["ok"])
        self.assertTrue(web.sensor.emergency_stop)

        bridge = mock.Mock()
        bridge.clear_fault.side_effect = RuntimeError("no acknowledgement")
        web.motor_bridge = bridge
        web.motor_bridge_connected = True
        with mock.patch.object(web, "connect_motor_bridge_if_needed", return_value=True):
            response = self.client.post("/api/clear_emergency", json={})
        self.assertEqual(response.status_code, 503)
        self.assertFalse(response.get_json()["ok"])
        self.assertTrue(web.sensor.emergency_stop)

    def test_disable_and_rom_start_do_not_clear_latched_estop(self):
        web.sensor.emergency_stop = True
        response = self.client.post("/api/set_motor", json={"enabled": False})
        self.assertTrue(response.get_json()["ok"])
        self.assertTrue(web.sensor.emergency_stop)
        response = self.client.post("/api/rom/start", json={})
        self.assertTrue(response.get_json()["ok"])
        self.assertTrue(web.sensor.emergency_stop)

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

    def test_server_accepts_shoulder_and_caps_lag_boost_at_55_pwm(self):
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
        self.assertEqual(web.sensor.output_limit, 55)

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

    def test_following_only_ignores_trajectory_and_demo_limit_caps_at_100(self):
        following = self.client.post("/api/apply_params", json={
            "controller_mode": "following_only",
            "target_mode": "step",
        })
        self.assertEqual(following.status_code, 200)
        self.assertEqual(web.sensor.controller_mode, "following_only")

        demo = self.client.post("/api/apply_params", json={
            "controller_mode": "trajectory_demo",
            "target_mode": "sine",
            "demo_output_limit": 999,
        })
        self.assertEqual(demo.status_code, 200)
        self.assertEqual(web.sensor.demo_output_limit, 100.0)

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
