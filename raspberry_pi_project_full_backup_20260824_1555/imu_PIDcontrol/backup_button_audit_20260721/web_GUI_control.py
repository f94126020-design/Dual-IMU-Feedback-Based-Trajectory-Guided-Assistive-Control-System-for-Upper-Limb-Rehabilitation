import csv
import os
import threading
import time
from datetime import datetime

from flask import Flask, jsonify, request, render_template_string

from imu_control import IMURehabSystem
from rom_calibration import ROMCalibrationSession
from muscle_allocator import MuscleAllocator, MusclePWM
from esp32motor import ESP32MotorBridge

app = Flask(__name__)

state_lock = threading.Lock()
sensor = None
reader_thread = None
reader_running = False
latest_frame = None
history = []
rom_calibration = ROMCalibrationSession()

system_status = "尚未初始化"
initialized = False
initializing = False
recording = False
csv_file = None
csv_writer = None
current_csv_path = None
record_count = 0
current_label = "CONTROL_TEST"
participant_id = "S01"
session_name = "session_01"
output_dir = os.path.abspath("control_data")

# ===== Motor output to ESP32 =====
# Stable CP2102 device path observed on this Raspberry Pi.
ESP32_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
ENABLE_AUTO_CONNECT_ESP32 = True  # safe in dry-run: reader loop can only send STOP
MOTOR_PWM_LIMIT = 255
MOTOR_CONTROL_PROFILE = "three_muscle"
# Deliberate hardware commissioning lock. Keep False until all motor directions,
# limit/encoder inputs and an accessible physical emergency stop are verified.
LIVE_MOTOR_OUTPUT_ALLOWED = False

motor_bridge = None
motor_bridge_connected = False
motor_bridge_error = ""

muscle_allocator = MuscleAllocator(
    pwm_limit=MOTOR_PWM_LIMIT,
    active_min_pwm=0,
    antagonist_release_gain=0.8,
    shoulder_release_pwm=60,
    command_filter_tau=0.18,
    wind_slew_rate=120.0,
    release_slew_rate=160.0,
    reverse_deadtime=0.15,
    # Motor 1: biceps, Motor 2: triceps, Motor 3: deltoid
    # If one motor direction is reversed, change its sign to -1.
    motor_sign=(1, 1, 1),
)

HTML_PAGE = r"""
<!DOCTYPE html>
<html lang="zh-Hant">
<head>
  <meta charset="UTF-8">
  <title>IMU 上肢復健軌跡控制系統</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    body{font-family:Arial,"Microsoft JhengHei",sans-serif;margin:0;background:#f3f4f6;color:#111827}
    header{background:#111827;color:white;padding:16px 24px} header h1{margin:0;font-size:24px} header p{margin:6px 0 0;color:#d1d5db}
    .container{display:grid;grid-template-columns:390px 1fr;gap:16px;padding:16px}.card{background:white;border-radius:12px;padding:16px;box-shadow:0 2px 8px rgba(0,0,0,.08);margin-bottom:16px}.card h2{font-size:18px;margin:0 0 12px}label{display:block;font-size:13px;color:#374151;margin:7px 0 4px}input,select{width:100%;box-sizing:border-box;padding:8px;border:1px solid #cbd5e1;border-radius:8px}button{width:100%;padding:10px;border:0;border-radius:8px;margin:5px 0;background:#2563eb;color:white;font-size:15px;cursor:pointer}button:hover{background:#1d4ed8}button:disabled{background:#9ca3af;cursor:not-allowed}.secondary{background:#64748b}.success{background:#16a34a}.danger{background:#dc2626}.warning{background:#f59e0b;color:#111827}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.status{background:#eef2ff;border-left:5px solid #4f46e5;border-radius:8px;padding:10px;white-space:pre-wrap;font-size:14px}.validation{background:#f8fafc;border:1px solid #d1d5db;border-radius:8px;padding:10px;margin:8px 0;white-space:pre-wrap;font-size:13px;line-height:1.5}.protocol{background:#fff7ed;border:1px solid #fdba74;border-radius:8px;padding:11px;margin:8px 0;white-space:pre-wrap;font-size:13px;line-height:1.55}.progress-track{height:10px;background:#e5e7eb;border-radius:999px;overflow:hidden;margin:8px 0}.progress-bar{height:100%;width:0;background:#2563eb;transition:width .15s linear}.live-angle{background:#ecfeff;border:1px solid #67e8f9;border-radius:8px;padding:9px;margin:8px 0;white-space:pre-wrap;font-family:Consolas,monospace;font-size:12px}.pass{color:#15803d;font-weight:700}.fail{color:#b91c1c;font-weight:700}.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}.metric{background:#f8fafc;border:1px solid #e5e7eb;border-radius:10px;padding:12px}.metric-title{font-size:13px;color:#64748b}.metric-value{font-size:22px;font-weight:700;margin-top:6px}.small{font-size:13px;color:#64748b;line-height:1.45}.rec{color:#dc2626;font-weight:700}.idle{color:#64748b;font-weight:700}canvas{width:100%;height:430px;background:white;border:1px solid #e5e7eb;border-radius:10px}@media(max-width:900px){.container{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(2,1fr)}}
  </style>
</head>
<body>
<header>
  <h1>雙 IMU 目標軌跡導引式上肢復健控制系統</h1>
  <p>雙 IMU 肘／肩關節控制空跑：可觀察三路 Desired PWM，ESP32 實際仍僅接收 STOP</p>
</header>
<div class="container">
  <div>
    <div class="card">
      <h2>1. 初始化</h2>
      <button onclick="initializeSystem()">初始化並校正 IMU</button>
      <button class="secondary" onclick="refreshStatus()">更新狀態</button>
      <div class="status" id="statusBox">尚未初始化</div>
    </div>

    <div class="card">
      <h2>2. 病患個人化活動範圍 ROM</h2>
      <p class="small">完成自然下垂重力校正後，依序記錄最大肘屈曲、肘 90° 到伸直、最大前平舉與最大側平舉。只建立當次個人化範圍，不判定 PASS／FAIL。</p>
      <button onclick="startValidation()">開始／重新記錄個人化 ROM</button>
      <button id="validationStageButton" class="secondary" onclick="prepareAndStartValidationStage()" disabled>2 秒準備後開始目前階段</button>
      <button id="validationApplyButton" class="success" onclick="applyValidation()" disabled>確認並套用個人化 ROM</button>
      <div class="validation" id="validationGuide">請先以手臂自然下垂姿勢初始化並校正 IMU。</div>
      <div class="progress-track"><div class="progress-bar" id="validationProgressBar"></div></div>
      <div class="protocol" id="validationProtocol">開始 ROM 後，這裡會顯示每個動作的起始姿勢與節奏。</div>
      <div class="live-angle" id="validationLiveAngles">即時角度：尚未初始化</div>
      <div class="validation" id="validationResults">尚無個人化 ROM 記錄。</div>
    </div>

    <div class="card">
      <h2>3. 復健任務設定</h2>
      <label>目標關節 Target Joint</label>
      <select id="targetJointInput" onchange="updateJointLabels()">
        <option value="elbow" selected>肘關節 elbow</option>
        <option value="shoulder">肩關節 shoulder</option>
      </select>

      <label>復健軌跡 Trajectory</label>
      <select id="targetModeInput">
        <option value="fixed" selected>固定角度 fixed target</option>
        <option value="sine">正弦軌跡跟隨 sine tracking</option>
        <option value="step">階躍測試 step response</option>
      </select>

      <div class="grid2">
        <div><label>固定目標角度</label><input id="fixedTargetInput" value="60"></div>
        <div><label>最小角度</label><input id="trajectoryMinInput" value="30"></div>
        <div><label>最大角度</label><input id="trajectoryMaxInput" value="90"></div>
        <div><label>正弦週期 秒</label><input id="trajectoryPeriodInput" value="5"></div>
        <div><label>階躍保持 秒</label><input id="stepHoldInput" value="3"></div>
        <div><label>Deadband 度</label><input id="deadbandInput" value="3"></div>
        <div><label>Trial label</label><input id="labelInput" value="elbow_sine_PID"></div>
      </div>
      <p class="small">可選擇肘或肩關節並觀察三路 Desired PWM；馬達尚未接線，因此實際 Output 仍固定為 0。</p>
    </div>

    <div class="card">
      <h2>4. 控制器設定</h2>
      <label>Controller Mode</label>
      <select id="controllerModeInput">
        <option value="p">P</option>
        <option value="pi">PI</option>
        <option value="pid">PID</option>
        <option value="assist" selected>assist-as-needed</option>
      </select>
      <div class="grid2">
        <div><label>Kp</label><input id="kpInput" value="0.8"></div>
        <div><label>Ki</label><input id="kiInput" value="0.02"></div>
        <div><label>Kd</label><input id="kdInput" value="0.05"></div>
        <div><label>控制器輸出限制</label><input id="outputLimitInput" value="50"></div>
        <div><label>Assist delay 秒</label><input id="assistDelayInput" value="0.25"></div>
        <div><label>積分限制</label><input id="integralLimitInput" value="100"></div>
      </div>
      <button onclick="applyParams()">套用關節、軌跡與控制參數</button>
    </div>

    <div class="card">
      <h2>5. 馬達安全控制</h2>
      <div class="validation fail" id="dryRunBanner">方案 B DRY RUN：肘肌反向 1:0.8、PWM 平滑與肩部漸進放線已啟用；實際 Output 固定為 0。</div>
      <div class="grid2">
        <button class="success" onclick="setMotor(true)">Enable Motor</button>
        <button class="secondary" onclick="setMotor(false)">Disable Motor</button>
      </div>
      <button class="danger" onclick="emergencyStop()">Emergency Stop</button>
      <button class="warning" onclick="clearEmergency()">解除急停</button>
      <p class="small" id="motorInterlockText">硬體輸出鎖尚未解除；Enable Motor 會被伺服器拒絕。</p>
    </div>

    <div class="card">
      <h2>6. 資料紀錄</h2>
      <label>受試者 ID</label><input id="participantInput" value="S01">
      <label>Session</label><input id="sessionInput" value="control_session_01">
      <label>輸出資料夾</label><input id="outputDirInput" value="control_data">
      <button class="success" onclick="startRecording()">開始錄製 CSV</button>
      <button class="danger" onclick="stopRecording()">停止錄製 CSV</button>
      <p id="recordingState" class="idle">未錄製</p>
      <p class="small" id="csvPathText"></p>
    </div>
  </div>

  <div>
    <div class="card">
      <h2>即時資料</h2>
      <div class="metrics">
        <div class="metric"><div class="metric-title" id="measuredTitle">測量角度 measured</div><div class="metric-value" id="measuredAngle">--</div></div>
        <div class="metric"><div class="metric-title">目標 target</div><div class="metric-value" id="targetAngle">--</div></div>
        <div class="metric"><div class="metric-title">誤差 error</div><div class="metric-value" id="errorValue">--</div></div>
        <div class="metric"><div class="metric-title">Servo Cmd</div><div class="metric-value" id="servoCmd">--</div></div>
        <div class="metric"><div class="metric-title">二頭 Desired</div><div class="metric-value" id="desiredPwmBiceps">--</div></div>
        <div class="metric"><div class="metric-title">三頭 Desired</div><div class="metric-value" id="desiredPwmTriceps">--</div></div>
        <div class="metric"><div class="metric-title">三角 Desired</div><div class="metric-value" id="desiredPwmDeltoid">--</div></div>
        <div class="metric"><div class="metric-title">二頭 Output</div><div class="metric-value" id="pwmBiceps">--</div></div>
        <div class="metric"><div class="metric-title">三頭 Output</div><div class="metric-value" id="pwmTriceps">--</div></div>
        <div class="metric"><div class="metric-title">三角 Output</div><div class="metric-value" id="pwmDeltoid">--</div></div>
        <div class="metric"><div class="metric-title">肘角 elbow</div><div class="metric-value" id="elbowAngle">--</div></div>
        <div class="metric"><div class="metric-title">肩角 shoulder</div><div class="metric-value" id="shoulderAngle">--</div></div>
        <div class="metric"><div class="metric-title">PID output</div><div class="metric-value" id="pidOutput">--</div></div>
        <div class="metric"><div class="metric-title">肩部放線目標</div><div class="metric-value" id="shoulderReleasePwm">--</div></div>
        <div class="metric"><div class="metric-title">控制遲滯</div><div class="metric-value" id="controlActive" style="font-size:16px">--</div></div>
        <div class="metric"><div class="metric-title">Anti-windup</div><div class="metric-value" id="antiWindup" style="font-size:16px">--</div></div>
        <div class="metric"><div class="metric-title">狀態</div><div class="metric-value" id="motionState" style="font-size:16px">--</div></div>
      </div>
      <p class="small" id="serialDebugText">Serial: --</p>
    </div>
    <div class="card">
      <h2>Target vs Measured</h2>
      <canvas id="angleCanvas" width="950" height="520"></canvas>
      <p class="small">上圖：灰線為未來目標軌跡預覽，藍線為病人已完成的 measured angle。下圖：紅線為真實 tracking error，0° 誤差線固定在中間，不再用 60° 平移顯示。垂直線為目前時間，右側可預先看到等等要跟隨的軌跡。CSV 仍會完整紀錄所有動態資料。</p>
    </div>
  </div>
</div>
<script>
let measuredHistory = [];
let errorHistory = [];
let timeHistory = [];
let latestTime = 0;
let validationFinishing = false;
let validationAutoRun = false;
let validationStageStarting = false;
let validationPreparing = false;

// 顯示視窗設定：目前時間會畫在圖中間偏左，右側保留未來目標軌跡給使用者預看。
const maxPoints = 1200;
const pastWindowSeconds = 8;
const futureWindowSeconds = 8;

function setText(id, text){document.getElementById(id).innerText=text;}
async function postJSON(url,data){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data||{})});return await r.json();}
async function getJSON(url){const r=await fetch(url);return await r.json();}
function fval(id, def){const v=parseFloat(document.getElementById(id).value); return Number.isFinite(v)?v:def;}
function sleep(ms){return new Promise(resolve=>setTimeout(resolve,ms));}

const validationStageDetails={
  elbow_flexion:{
    title:'動作 1｜手臂下垂－最大二頭肌屈曲',
    body:'起始：上臂自然垂在身側並保持不動，肘伸直，手掌朝向身體。\n0–1.5 秒：只彎曲手肘，到你舒服且無痛的最大屈曲。\n1.5–2.25 秒：保持最大屈曲。\n2.25–3.75 秒：伸直回到手臂下垂。\n3.75–5 秒：保持肘伸直。共做 2 次。\n系統會用保持區段估算最大屈曲並學習肘關節主要軸與方向。'
  },
  elbow_extension:{
    title:'動作 2｜肘 90°－三頭肌伸展至打直',
    body:'起始：上臂貼近身體，先把肘擺在約 90°。不要把手臂舉過頭。\n0–1.5 秒：用三頭肌把前臂向下伸展，直到手臂打直；若平常能無痛微過伸，可到自然的微過伸位置。\n1.5–2.25 秒：保持伸直終點。\n2.25–3.75 秒：彎回約 90°。\n3.75–5 秒：保持 90°。共做 2 次。\n不要用肩膀甩動，也不要強迫壓到過伸。'
  },
  shoulder_front:{
    title:'動作 3｜最大前平舉',
    body:'起始：手臂自然下垂，肘保持伸直，手掌朝內。\n0–1.5 秒：從正前方抬到舒服且無痛的最大角度。\n1.5–2.25 秒：保持最大前舉。\n2.25–3.75 秒：沿原路放下。\n3.75–5 秒：保持自然下垂。共做 2 次。\n避免彎肘、聳肩、後仰或用身體甩動。'
  },
  shoulder_side:{
    title:'動作 4｜最大側平舉',
    body:'起始：手臂自然下垂，肘保持伸直，軀幹直立。\n0–1.5 秒：從身體正側方抬到舒服且無痛的最大角度。\n1.5–2.25 秒：保持最大側舉。\n2.25–3.75 秒：沿原路放下。\n3.75–5 秒：保持自然下垂。共做 2 次。\n避免往前帶、聳肩、彎肘或身體側傾。'
  }
};

function showValidationProtocol(stage){
  const detail=validationStageDetails[stage];
  setText('validationProtocol',detail?(detail.title+'\n'+detail.body):'等待 ROM 記錄階段。');
}

function params(){return {target_joint:document.getElementById('targetJointInput').value,target_mode:document.getElementById('targetModeInput').value,fixed_target_angle:fval('fixedTargetInput',60),trajectory_min_angle:fval('trajectoryMinInput',30),trajectory_max_angle:fval('trajectoryMaxInput',90),trajectory_period:fval('trajectoryPeriodInput',5),step_hold_time:fval('stepHoldInput',3),controller_mode:document.getElementById('controllerModeInput').value,deadband:fval('deadbandInput',3),assist_delay:fval('assistDelayInput',.25),output_limit:Math.min(Math.abs(fval('outputLimitInput',50)),255),kp:fval('kpInput',.8),ki:fval('kiInput',.02),kd:fval('kdInput',.05),integral_limit:fval('integralLimitInput',100)};}

function resetPlot(){
  measuredHistory=[];
  errorHistory=[];
  timeHistory=[];
  latestTime=0;
  drawPlot();
}

function updateJointLabels(){
  const j=document.getElementById('targetJointInput').value;
  const mode=document.getElementById('targetModeInput').value;
  if(j==='shoulder'){
    setText('measuredTitle','肩關節 measured');
    document.getElementById('labelInput').value=(mode==='step')?'shoulder_step_PID':'shoulder_sine_PID';
  }else{
    setText('measuredTitle','肘關節 measured');
    document.getElementById('labelInput').value=(mode==='step')?'elbow_step_PID':'elbow_sine_PID';
  }
  resetPlot();
}

document.getElementById('targetModeInput').addEventListener('change', updateJointLabels);
document.getElementById('targetJointInput').addEventListener('change', updateJointLabels);
['trajectoryMinInput','trajectoryMaxInput','trajectoryPeriodInput','stepHoldInput'].forEach(id=>{
  document.getElementById(id).addEventListener('change', resetPlot);
});

function getTrajectoryCenter(){
  let minA=fval('trajectoryMinInput',30);
  let maxA=fval('trajectoryMaxInput',90);
  if(maxA<minA){const tmp=minA; minA=maxA; maxA=tmp;}
  return 0.5*(minA+maxA);
}

async function initializeSystem(){
  resetPlot();
  setText('statusBox','初始化與 5 秒校正中，請保持手臂自然下垂靜止...');
  setText('validationProtocol','5 秒校正姿勢\n站直或坐直，肩膀放鬆，手臂自然垂在身側。\n肘完全伸直，前臂保持中立、手掌朝向身體。\n校正期間不要移動、不要碰觸 IMU。');
  const res=await postJSON('/api/init',params());
  setText('statusBox',res.message);
}
async function startValidation(){
  const res=await postJSON('/api/rom/start',{});
  if(!res.ok){alert(res.message);return;}
  setText('statusBox',res.message);
  validationFinishing=false;
  validationAutoRun=true;
  await refreshStatus();
  await prepareAndStartValidationStage();
}
async function prepareAndStartValidationStage(){
  if(validationPreparing||validationStageStarting)return;
  const data=await getJSON('/api/status');
  const stage=(data.rom_calibration||{}).expected_stage;
  if(!stage)return;
  validationPreparing=true;
  showValidationProtocol(stage);
  document.getElementById('validationStageButton').disabled=true;
  document.getElementById('validationProgressBar').style.width='0%';
  for(let count=2;count>=1;count--){
    const detail=validationStageDetails[stage];
    setText('validationGuide',(detail?detail.title:stage)+'\n請先擺好起始姿勢，'+count+' 秒後自動開始。');
    await sleep(1000);
  }
  validationPreparing=false;
  await startValidationStage();
}
async function startValidationStage(){
  if(validationStageStarting)return;
  validationStageStarting=true;
  const res=await postJSON('/api/rom/start_stage',{});
  validationStageStarting=false;
  if(!res.ok){validationAutoRun=false;alert(res.message);return;}
  validationFinishing=false;
  setText('statusBox',res.message);
  await refreshStatus();
}
async function finishValidationStage(){
  if(validationFinishing)return;
  validationFinishing=true;
  const res=await postJSON('/api/rom/finish_stage',{});
  if(!res.ok)alert(res.message);
  else setText('statusBox',res.message);
  await refreshStatus();
  validationFinishing=false;
  if(res.ok && validationAutoRun && res.rom_calibration && res.rom_calibration.expected_stage){
    setTimeout(prepareAndStartValidationStage,400);
  }else if(res.ok){
    validationAutoRun=false;
  }
}
async function applyValidation(){
  const res=await postJSON('/api/rom/apply',{});
  if(!res.ok){alert(res.message);return;}
  if(res.rom){
    document.getElementById('trajectoryMinInput').value=fmtMetric(res.rom.elbow_extension_target_deg,1);
    document.getElementById('trajectoryMaxInput').value=fmtMetric(res.rom.elbow_flexion_target_deg,1);
    document.getElementById('fixedTargetInput').value=fmtMetric(res.rom.elbow_flexion_target_deg,1);
    resetPlot();
  }
  setText('statusBox',res.message);
  await refreshStatus();
}
async function refreshStatus(){updatePage(await getJSON('/api/status'));}
async function applyParams(){
  const res=await postJSON('/api/apply_params',params());
  alert(res.message);
  resetPlot();
}
async function setMotor(enabled){const res=await postJSON('/api/set_motor',{enabled}); setText('statusBox',res.message);}
async function emergencyStop(){const res=await postJSON('/api/emergency_stop',{}); setText('statusBox',res.message);}
async function clearEmergency(){const res=await postJSON('/api/clear_emergency',{}); setText('statusBox',res.message);}
async function startRecording(){
  resetPlot();
  const data={participant_id:document.getElementById('participantInput').value||'S01',session_name:document.getElementById('sessionInput').value||'session',output_dir:document.getElementById('outputDirInput').value||'control_data',label:document.getElementById('labelInput').value||'trial'};
  const res=await postJSON('/api/start_recording',data);
  if(!res.ok){alert(res.message);return;}
  setText('recordingState','錄製中');document.getElementById('recordingState').className='rec';setText('csvPathText',res.csv_path);
}
async function stopRecording(){const res=await postJSON('/api/stop_recording',{}); setText('recordingState','未錄製');document.getElementById('recordingState').className='idle';setText('csvPathText',res.message);}

function updatePage(data){
  setText('statusBox',data.status);
  updateValidationPanel(data.rom_calibration||{});
  if(data.motor_output){
    const s=data.motor_output;
    setText('dryRunBanner','方案 B '+String(s.mode).toUpperCase()+'：肘肌反向 1:'+s.antagonist_release_gain
      +'、低通 '+s.command_filter_tau+'s、換向停頓 '+Math.round(s.reverse_deadtime*1000)+'ms；實際輸出 '
      +(s.live_output_allowed?'已允許':'固定為 0，ESP32 只接收 STOP')+'。');
    setText('motorInterlockText',s.live_output_allowed?'硬體輸出鎖已解除。':('硬體輸出鎖未解除：'+s.lock_reason));
  }
  if(data.recording){setText('recordingState','錄製中，筆數：'+data.record_count);document.getElementById('recordingState').className='rec';setText('csvPathText',data.csv_path||'');}
  if(data.frame){
    const f=data.frame;
    setText('validationLiveAngles','即時角度｜肘='+f.elbow_angle.toFixed(1)+'°'
      +'｜roll候選='+f.elbow_roll_signed.toFixed(1)+'°｜pitch候選='+f.elbow_pitch_signed.toFixed(1)+'°'
      +'｜肩='+f.shoulder_angle.toFixed(1)+'°｜肩速度='+(f.shoulder_angle_velocity ?? 0).toFixed(1)+'°/s');
    const measured=(f.measured_angle!==undefined)?f.measured_angle:(f.target_joint==='shoulder'?f.shoulder_angle:f.elbow_angle);
    setText('measuredAngle',measured.toFixed(1)+'°');
    setText('elbowAngle',f.elbow_angle.toFixed(1)+'°');
    setText('shoulderAngle',f.shoulder_angle.toFixed(1)+'°');
    setText('targetAngle',f.target_angle.toFixed(1)+'°');
    setText('errorValue',f.error.toFixed(1)+'°');
    setText('servoCmd',f.servo_cmd.toFixed(1));
    setText('desiredPwmBiceps', (f.desired_pwm_biceps ?? 0).toFixed(0));
    setText('desiredPwmTriceps', (f.desired_pwm_triceps ?? 0).toFixed(0));
    setText('desiredPwmDeltoid', (f.desired_pwm_deltoid ?? 0).toFixed(0));
    setText('pwmBiceps', (f.pwm_biceps ?? 0).toFixed(0));
    setText('pwmTriceps', (f.pwm_triceps ?? 0).toFixed(0));
    setText('pwmDeltoid', (f.pwm_deltoid ?? 0).toFixed(0));
    const serialText = 'ESP32 connected=' + data.motor_bridge_connected
      + ' | TX=' + (f.serial_tx || '--')
      + ' | RX=' + (data.motor_bridge_last_line || '--')
      + ' | error=' + (data.motor_bridge_error || '--');
    setText('serialDebugText', serialText);
    setText('pidOutput',f.pid_output.toFixed(1));
    setText('shoulderReleasePwm',(f.shoulder_release_pwm ?? 0).toFixed(1));
    setText('controlActive',f.control_active?'ACTIVE':'IDLE');
    setText('antiWindup',f.anti_windup_active?'HOLD':'OFF');
    setText('motionState',f.motion_state);

    measuredHistory.push(measured);
    errorHistory.push(f.error);
    timeHistory.push(f.time);
    latestTime = f.time;

    if(measuredHistory.length>maxPoints){
      measuredHistory.shift();
      errorHistory.shift();
      timeHistory.shift();
    }
    drawPlot();
  }
}

function fmtMetric(value, digits=1){
  return (typeof value==='number' && Number.isFinite(value))?value.toFixed(digits):'--';
}
function updateValidationPanel(v){
  const stageButton=document.getElementById('validationStageButton');
  const applyButton=document.getElementById('validationApplyButton');

  if(validationPreparing){
    stageButton.disabled=true;
    applyButton.disabled=true;
    return;
  }

  if(!v.state || v.state==='idle'){
    setText('validationGuide','請先以手臂自然下垂姿勢初始化並完成重力校正，再開始個人化 ROM。');
    setText('validationResults','尚無個人化 ROM 記錄。');
    document.getElementById('validationProgressBar').style.width='0%';
    stageButton.disabled=true;applyButton.disabled=true;return;
  }

  if(v.current_stage){
    showValidationProtocol(v.current_stage);
    const duration=10;
    const progress=Math.max(0,Math.min(100,100*(v.elapsed||0)/duration));
    document.getElementById('validationProgressBar').style.width=progress.toFixed(1)+'%';
    const guide='現在執行：'+v.current_stage_label+'\n目前動作：'+(v.phase||'')
      +'\n剩餘 '+fmtMetric(v.remaining,1)+' 秒｜樣本 '+(v.sample_count||0);
    setText('validationGuide',guide);
    stageButton.disabled=true;
    if(v.remaining<=0.05)finishValidationStage();
  }else if(v.expected_stage){
    showValidationProtocol(v.expected_stage);
    document.getElementById('validationProgressBar').style.width='0%';
    setText('validationGuide','下一階段：'+v.expected_stage_label+'\n先閱讀下方完整動作內容，再按「2 秒準備後開始目前階段」。');
    stageButton.disabled=false;
  }else if(v.ready_to_apply && !v.applied){
    document.getElementById('validationProgressBar').style.width='100%';
    setText('validationGuide','四個 ROM 動作已記錄。請檢查數值後，按「確認並套用個人化 ROM」。');
    stageButton.disabled=true;
  }else if(v.applied){
    document.getElementById('validationProgressBar').style.width='100%';
    setText('validationGuide','病患個人化 ROM 已套用，本次初始化有效。');
    stageButton.disabled=true;
  }else{
    document.getElementById('validationProgressBar').style.width='100%';
    setText('validationGuide','ROM 記錄不完整，請重新開始缺少的動作。');
    stageButton.disabled=true;
  }
  applyButton.disabled=!v.ready_to_apply||v.applied;

  let lines=[];
  const results=v.results||{};
  if(results.elbow_flexion){
    const r=results.elbow_flexion;
    lines.push('已記錄｜最大肘屈曲：'+fmtMetric(r.flexion_max_deg)+'°｜軸='+r.elbow_axis+'｜sign='+r.elbow_sign);
  }
  if(results.elbow_extension){
    const r=results.elbow_extension;
    lines.push('已記錄｜肘伸展終點：'+fmtMetric(r.extension_angle_deg)+'°｜90°起始量測：'+fmtMetric(r.start_90_angle_deg)+'°');
  }
  if(results.shoulder_front){
    const r=results.shoulder_front;
    lines.push('已記錄｜最大前平舉：'+fmtMetric(r.maximum_deg)+'°｜回到下垂：'+fmtMetric(r.return_angle_deg)+'°');
  }
  if(results.shoulder_side){
    const r=results.shoulder_side;
    lines.push('已記錄｜最大側平舉：'+fmtMetric(r.maximum_deg)+'°｜回到下垂：'+fmtMetric(r.return_angle_deg)+'°');
  }
  if(v.rom){
    const r=v.rom;
    lines.push('建議 90% 訓練目標：');
    lines.push('  肘屈曲 '+fmtMetric(r.elbow_flexion_target_deg)+'°｜肘伸展 '+fmtMetric(r.elbow_extension_target_deg)+'°');
    lines.push('  前平舉 '+fmtMetric(r.shoulder_front_target_deg)+'°｜側平舉 '+fmtMetric(r.shoulder_side_target_deg)+'°');
  }
  (v.warnings||[]).forEach(warning=>lines.push('注意：'+warning));
  setText('validationResults',lines.length?lines.join('\n'):'等待第一個 ROM 動作。');
}

function targetAtTime(t){
  let minA=fval('trajectoryMinInput',30);
  let maxA=fval('trajectoryMaxInput',90);
  if(maxA<minA){const tmp=minA; minA=maxA; maxA=tmp;}
  const center=0.5*(minA+maxA);
  const amp=0.5*(maxA-minA);
  const mode=document.getElementById('targetModeInput').value;

  if(mode==='fixed') return fval('fixedTargetInput',60);

  if(mode==='step'){
    const hold=Math.max(fval('stepHoldInput',3),0.1);
    const idx=Math.floor(Math.max(t,0)/hold);
    return (idx%2===1)?maxA:minA;
  }

  const period=Math.max(fval('trajectoryPeriodInput',5),0.1);
  return center + amp*Math.sin(2*Math.PI*Math.max(t,0)/period);
}

function drawPlot(){
  const c=document.getElementById('angleCanvas'),ctx=c.getContext('2d'),w=c.width,h=c.height;
  ctx.clearRect(0,0,w,h);

  // 共用時間視窗：左邊顯示已發生資料，右邊顯示未來 target preview
  const currentTime = latestTime || 0;
  const windowStart = Math.max(0, currentTime - pastWindowSeconds);
  const windowEnd = currentTime + futureWindowSeconds;
  const windowSpan = Math.max(windowEnd - windowStart, 0.1);

  // 版面切成上下兩張圖：上面角度，下面誤差
  const ml=55,mr=20,topMt=20,topH=285,gap=42,errMt=topMt+topH+gap,errH=135,mb=35;
  const pw=w-ml-mr;
  const angleYMin=0, angleYMax=120;

  let minA=fval('trajectoryMinInput',30);
  let maxA=fval('trajectoryMaxInput',90);
  if(maxA<minA){const tmp=minA; minA=maxA; maxA=tmp;}
  // error 使用獨立 y 軸，0° 在中間。範圍依目標角度自動給足。
  const errorLimit=Math.max(30, Math.ceil(Math.max(Math.abs(minA), Math.abs(maxA), Math.abs(maxA-minA))/10)*10);

  function mxTime(t){return ml + ((t-windowStart)/windowSpan)*pw;}
  function myAngle(v){v=Math.max(angleYMin,Math.min(v,angleYMax));return topMt+topH-(v-angleYMin)/(angleYMax-angleYMin)*topH;}
  function myErr(v){v=Math.max(-errorLimit,Math.min(v,errorLimit));return errMt+errH-(v+errorLimit)/(2*errorLimit)*errH;}

  ctx.font='12px Arial';

  // ---------- 上圖：target / measured angle ----------
  ctx.strokeStyle='#d1d5db';ctx.strokeRect(ml,topMt,pw,topH);
  ctx.fillStyle='#374151';ctx.fillText('Angle (deg)',ml,topMt-7);

  ctx.strokeStyle='#eef2f7';ctx.fillStyle='#6b7280';
  [0,30,60,90,120].forEach(a=>{
    const y=myAngle(a);
    ctx.beginPath();ctx.moveTo(ml,y);ctx.lineTo(ml+pw,y);ctx.stroke();
    ctx.fillText(String(a),ml-35,y+4);
  });

  const xNow = mxTime(currentTime);
  ctx.strokeStyle='#111827';ctx.lineWidth=1;ctx.setLineDash([5,5]);
  ctx.beginPath();ctx.moveTo(xNow,topMt);ctx.lineTo(xNow,topMt+topH);ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle='#111827';ctx.fillText('現在',xNow+5,topMt+topH-8);

  // 灰線：完整目標軌跡預覽，包含過去與未來
  ctx.strokeStyle='#9ca3af';ctx.lineWidth=2;ctx.beginPath();
  const samples=260;
  for(let i=0;i<samples;i++){
    const tt = windowStart + (i/(samples-1))*windowSpan;
    const x = mxTime(tt);
    const y = myAngle(targetAtTime(tt));
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();

  function drawAngleSeries(times, vals, color, width){
    if(times.length<2)return;
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();
    let started=false;
    for(let i=0;i<times.length;i++){
      const t=times[i];
      if(t<windowStart || t>currentTime) continue;
      const x=mxTime(t), y=myAngle(vals[i]);
      if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
    }
    if(started) ctx.stroke();
  }
  drawAngleSeries(timeHistory, measuredHistory, '#2563eb', 3);

  ctx.fillStyle='#2563eb';ctx.fillText('Measured selected joint',ml+10,topMt+18);
  ctx.fillStyle='#6b7280';ctx.fillText('Target preview',ml+190,topMt+18);

  // ---------- 下圖：tracking error，獨立 y 軸 ----------
  ctx.strokeStyle='#d1d5db';ctx.strokeRect(ml,errMt,pw,errH);
  ctx.fillStyle='#374151';ctx.fillText('Tracking error (deg)',ml,errMt-7);

  // Error 水平格線：+limit、0、-limit。0° 誤差線在中間。
  ctx.strokeStyle='#eef2f7';ctx.fillStyle='#6b7280';
  [-errorLimit,0,errorLimit].forEach(e=>{
    const y=myErr(e);
    ctx.beginPath();ctx.moveTo(ml,y);ctx.lineTo(ml+pw,y);ctx.stroke();
    ctx.fillText(String(e),ml-42,y+4);
  });

  // 0 誤差線加粗，讓使用者清楚知道紅線貼近這條線就是跟上目標
  ctx.strokeStyle='#9ca3af';ctx.lineWidth=1.2;
  ctx.beginPath();ctx.moveTo(ml,myErr(0));ctx.lineTo(ml+pw,myErr(0));ctx.stroke();

  // 目前時間線也畫在 error 圖
  ctx.strokeStyle='#111827';ctx.lineWidth=1;ctx.setLineDash([5,5]);
  ctx.beginPath();ctx.moveTo(xNow,errMt);ctx.lineTo(xNow,errMt+errH);ctx.stroke();
  ctx.setLineDash([]);

  function drawErrorSeries(times, vals, color, width){
    if(times.length<2)return;
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();
    let started=false;
    for(let i=0;i<times.length;i++){
      const t=times[i];
      if(t<windowStart || t>currentTime) continue;
      const x=mxTime(t), y=myErr(vals[i]);
      if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
    }
    if(started) ctx.stroke();
  }
  drawErrorSeries(timeHistory, errorHistory, '#dc2626', 2.2);

  ctx.fillStyle='#dc2626';ctx.fillText('Error = target - measured',ml+10,errMt+18);
  ctx.fillStyle='#6b7280';ctx.fillText('0° error',ml+150,myErr(0)-6);

  // 時間標籤
  ctx.fillStyle='#6b7280';
  ctx.fillText(windowStart.toFixed(1)+'s',ml,errMt+errH+22);
  ctx.fillText(currentTime.toFixed(1)+'s',xNow-12,errMt+errH+22);
  ctx.fillText(windowEnd.toFixed(1)+'s',ml+pw-50,errMt+errH+22);
}


setInterval(async()=>{try{updatePage(await getJSON('/api/status'));}catch(e){setText('statusBox','無法連線到 Flask Server');}},100);
updateJointLabels();
drawPlot();
</script>
</body>
</html>
"""

def safe_float(value, default):
    try:
        return float(value)
    except Exception:
        return default

def safe_bool(value):
    return bool(value)

def set_status(message):
    global system_status
    with state_lock:
        system_status = message

def serialize_frame(frame):
    if frame is None:
        return None
    keys_float = [
        "time","upper_angle","forearm_angle","signed_elbow_angle","elbow_roll_signed","elbow_pitch_signed","elbow_sign","elbow_angle_raw","elbow_angle",
        "measured_angle","measured_angle_raw","upper_roll","upper_pitch","forearm_roll","forearm_pitch",
        "omega","alpha","jerk","shoulder_angle","shoulder_angle_raw","shoulder_angle_velocity","shoulder_release_pwm",
        "shoulder_dx","shoulder_dy","shoulder_dz","target_angle","error","error_for_control","error_active_time",
        "integral_error","derivative_error","filtered_measurement_velocity","raw_pid_output","pid_output","motor_cmd","servo_cmd",
        "deadband_on","deadband_off",
        "desired_motor_cmd","desired_pwm_biceps","desired_pwm_triceps","desired_pwm_deltoid",
        "pwm_biceps","pwm_triceps","pwm_deltoid",
        "encoder_count_biceps","encoder_count_triceps","encoder_count_deltoid",
        "encoder_vel_biceps","encoder_vel_triceps","encoder_vel_deltoid"
    ]
    out = {}
    for k in keys_float:
        try:
            out[k] = float(frame.get(k, 0.0))
        except Exception:
            out[k] = 0.0
    for k in ["motion_state","shoulder_motion_state","shoulder_plane","elbow_axis","target_mode","target_joint","controller_mode","serial_tx"]:
        out[k] = str(frame.get(k, ""))
    for k in ["shoulder_assist","shoulder_release_command","shoulder_motor_enable","shoulder_accel_valid","control_active","anti_windup_active","motor_enabled","emergency_stop"]:
        out[k] = bool(frame.get(k, False))
    return out

def apply_params_to_sensor(data, reset_controller=True):
    global sensor
    if sensor is None:
        return
    params = {
        "target_joint": data.get("target_joint", "elbow"),
        "target_mode": data.get("target_mode", "fixed"),
        "fixed_target_angle": safe_float(data.get("fixed_target_angle"), 60.0),
        "trajectory_min_angle": safe_float(data.get("trajectory_min_angle"), 30.0),
        "trajectory_max_angle": safe_float(data.get("trajectory_max_angle"), 90.0),
        "trajectory_period": safe_float(data.get("trajectory_period"), 5.0),
        "step_hold_time": safe_float(data.get("step_hold_time"), 3.0),
        "controller_mode": data.get("controller_mode", "assist"),
        "deadband": safe_float(data.get("deadband"), 3.0),
        "deadband_on": safe_float(data.get("deadband"), 3.0) + 1.0,
        "deadband_off": max(0.0, safe_float(data.get("deadband"), 3.0) - 1.0),
        "assist_delay": safe_float(data.get("assist_delay"), 0.25),
        "output_limit": min(abs(safe_float(data.get("output_limit"), 50.0)), MOTOR_PWM_LIMIT),
        "kp": safe_float(data.get("kp"), 0.8),
        "ki": safe_float(data.get("ki"), 0.02),
        "kd": safe_float(data.get("kd"), 0.05),
        "integral_limit": safe_float(data.get("integral_limit"), 100.0),
    }
    if hasattr(sensor, "configure_control"):
        sensor.configure_control(reset_controller=reset_controller, **params)
    else:
        for k, v in params.items():
            setattr(sensor, k, v)

def connect_motor_bridge_if_needed():
    global motor_bridge, motor_bridge_connected, motor_bridge_error

    if motor_bridge_connected and motor_bridge is not None:
        return True

    try:
        motor_bridge = ESP32MotorBridge(
            port=ESP32_PORT,
            baudrate=115200,
            pwm_limit=MOTOR_PWM_LIMIT,
        )
        motor_bridge.connect()
        motor_bridge_connected = True
        motor_bridge_error = ""
        return True
    except Exception as e:
        motor_bridge_connected = False
        motor_bridge_error = str(e)
        return False

def stop_motor_bridge_safely():
    global motor_bridge, motor_bridge_connected, motor_bridge_error
    try:
        if motor_bridge is not None:
            motor_bridge.stop()
    except Exception as e:
        motor_bridge_error = str(e)
        motor_bridge_connected = False


def reader_loop():
    global latest_frame, record_count, motor_bridge_connected, motor_bridge_error

    while reader_running:
        try:
            # 1) Read IMU + calculate target/error/PID motor_cmd
            frame = sensor.read_frame()
            rom_calibration.add_sample(frame)

            # 2) Read ESP32 encoder feedback if connected
            fb = None
            if motor_bridge_connected and motor_bridge is not None:
                try:
                    fb = motor_bridge.get_feedback()
                except Exception as e:
                    motor_bridge_error = str(e)
                    motor_bridge_connected = False
                    fb = None

            # 3) Allocate joint-level control output to 3 muscle motors.
            # desired_pwm: 只用來顯示「如果啟用馬達，控制器想輸出什麼」。
            # output_pwm: 真正會送到 ESP32；只有 motor_enabled=True 且沒有急停時才非 0。
            desired_cmd = frame.get("pid_output", 0)
            desired_pwm = muscle_allocator.allocate(
                target_joint=frame.get("target_joint", "elbow"),
                motor_cmd=desired_cmd,
                control_profile=MOTOR_CONTROL_PROFILE,
                motor_enabled=True,
                emergency_stop=False,
                shoulder_release_command=frame.get("shoulder_release_command", False),
                shoulder_release_pwm=frame.get("shoulder_release_pwm", 0.0),
                shoulder_motor_enable=frame.get("shoulder_motor_enable", True),
                feedback=fb,
                now=frame.get("time"),
            )

            can_output = (
                LIVE_MOTOR_OUTPUT_ALLOWED
                and frame.get("motor_enabled", False)
                and not frame.get("emergency_stop", False)
            )
            output_pwm = desired_pwm if can_output else MusclePWM()

            frame["desired_motor_cmd"] = desired_cmd
            frame["desired_pwm_biceps"] = desired_pwm.biceps
            frame["desired_pwm_triceps"] = desired_pwm.triceps
            frame["desired_pwm_deltoid"] = desired_pwm.deltoid

            frame["pwm_biceps"] = output_pwm.biceps
            frame["pwm_triceps"] = output_pwm.triceps
            frame["pwm_deltoid"] = output_pwm.deltoid

            if fb is not None:
                frame["encoder_count_biceps"] = fb.count1
                frame["encoder_count_triceps"] = fb.count2
                frame["encoder_count_deltoid"] = fb.count3
                frame["encoder_vel_biceps"] = fb.vel1
                frame["encoder_vel_triceps"] = fb.vel2
                frame["encoder_vel_deltoid"] = fb.vel3
            else:
                frame["encoder_count_biceps"] = 0
                frame["encoder_count_triceps"] = 0
                frame["encoder_count_deltoid"] = 0
                frame["encoder_vel_biceps"] = 0.0
                frame["encoder_vel_triceps"] = 0.0
                frame["encoder_vel_deltoid"] = 0.0

            # 4) Send PWM to ESP32 only when motor is enabled and ESP32 is connected
            if motor_bridge_connected and motor_bridge is not None:
                try:
                    if (not LIVE_MOTOR_OUTPUT_ALLOWED
                            or frame.get("emergency_stop", False)
                            or not frame.get("motor_enabled", False)):
                        frame["serial_tx"] = "STOP"
                        motor_bridge.stop()
                    else:
                        frame["serial_tx"] = f"PWM,{output_pwm.biceps},{output_pwm.triceps},{output_pwm.deltoid}"
                        motor_bridge.set_pwm(output_pwm.biceps, output_pwm.triceps, output_pwm.deltoid)
                except Exception as e:
                    motor_bridge_error = str(e)
                    motor_bridge_connected = False

            if "serial_tx" not in frame:
                frame["serial_tx"] = "ESP32_NOT_CONNECTED" if not motor_bridge_connected else "NO_COMMAND"

            # 5) Save latest frame and CSV
            with state_lock:
                latest_frame = frame
                history.append(frame)
                if len(history) > 500:
                    history.pop(0)

                if recording and csv_writer is not None:
                    csv_writer.writerow([
                        datetime.now().isoformat(timespec="milliseconds"),
                        frame.get("time"), participant_id, session_name, current_label,
                        frame.get("target_joint"), frame.get("target_mode"), frame.get("controller_mode"),
                        frame.get("target_angle"), frame.get("measured_angle"), frame.get("measured_angle_raw"), frame.get("error"),
                        frame.get("elbow_angle"), frame.get("elbow_angle_raw"), frame.get("shoulder_angle"), frame.get("shoulder_angle_raw"),
                        frame.get("error_for_control"), frame.get("integral_error"), frame.get("derivative_error"),
                        frame.get("filtered_measurement_velocity"), frame.get("control_active"), frame.get("anti_windup_active"),
                        frame.get("raw_pid_output"), frame.get("pid_output"), frame.get("motor_cmd"), frame.get("servo_cmd"),
                        frame.get("desired_motor_cmd"), frame.get("desired_pwm_biceps"), frame.get("desired_pwm_triceps"), frame.get("desired_pwm_deltoid"),
                        frame.get("pwm_biceps"), frame.get("pwm_triceps"), frame.get("pwm_deltoid"),
                        frame.get("encoder_count_biceps"), frame.get("encoder_count_triceps"), frame.get("encoder_count_deltoid"),
                        frame.get("encoder_vel_biceps"), frame.get("encoder_vel_triceps"), frame.get("encoder_vel_deltoid"),
                        frame.get("upper_angle"), frame.get("forearm_angle"), frame.get("omega"), frame.get("alpha"), frame.get("jerk"),
                        frame.get("shoulder_motion_state"), frame.get("shoulder_release_command"), frame.get("shoulder_release_pwm"), frame.get("shoulder_motor_enable"),
                        frame.get("motor_enabled"), frame.get("emergency_stop"), frame.get("motion_state"),
                    ])
                    record_count += 1
                    if record_count % 50 == 0:
                        csv_file.flush()

            time.sleep(0.01)

        except Exception as e:
            set_status(f"讀取 IMU / 馬達資料錯誤: {e}")
            stop_motor_bridge_safely()
            time.sleep(0.1)

def init_worker(params):
    global sensor, reader_thread, reader_running, initialized, initializing
    try:
        set_status("正在建立 IMU 系統...")
        new_sensor = IMURehabSystem(
            target_joint=params.get("target_joint", "elbow"),
            target_mode=params.get("target_mode", "fixed"),
            fixed_target_angle=safe_float(params.get("fixed_target_angle"), 60.0),
            trajectory_min_angle=safe_float(params.get("trajectory_min_angle"), 30.0),
            trajectory_max_angle=safe_float(params.get("trajectory_max_angle"), 90.0),
            trajectory_period=safe_float(params.get("trajectory_period"), 5.0),
            step_hold_time=safe_float(params.get("step_hold_time"), 3.0),
            controller_mode=params.get("controller_mode", "assist"),
            deadband=safe_float(params.get("deadband"), 3.0),
            deadband_on=safe_float(params.get("deadband"), 3.0) + 1.0,
            deadband_off=max(0.0, safe_float(params.get("deadband"), 3.0) - 1.0),
            assist_delay=safe_float(params.get("assist_delay"), 0.25),
            output_limit=min(abs(safe_float(params.get("output_limit"), 50.0)), MOTOR_PWM_LIMIT),
            kp=safe_float(params.get("kp"), 0.8),
            ki=safe_float(params.get("ki"), 0.02),
            kd=safe_float(params.get("kd"), 0.05),
            integral_limit=safe_float(params.get("integral_limit"), 100.0),
            motor_enabled=False,
            emergency_stop=False,
        )
        set_status("正在掃描與初始化 MPU6050...")
        channels = new_sensor.scan_and_init()
        set_status(f"初始化成功，可用通道: {channels}\n請保持手臂自然下垂並靜止，開始 5 秒校正。")
        def progress(elapsed, remaining):
            set_status(f"校正中... 剩餘 {remaining:.1f} 秒")
        result = new_sensor.calibrate(seconds=5, progress_callback=progress)
        muscle_allocator.reset_conditioner()
        with state_lock:
            sensor = new_sensor
            rom_calibration.reset()
            initialized = True
            initializing = False
        if ENABLE_AUTO_CONNECT_ESP32:
            ok = connect_motor_bridge_if_needed()
            if ok:
                set_status(f"校正完成，樣本數: {result['sample_count']}。\nESP32 motor bridge 已連線。\n可開始記錄病患個人化 ROM；馬達仍為 DRY RUN。")
            else:
                set_status(f"校正完成，樣本數: {result['sample_count']}。\nESP32 尚未連線：{motor_bridge_error}\n可先記錄病患個人化 ROM；馬達仍為 DRY RUN。")
        else:
            set_status(f"校正完成，樣本數: {result['sample_count']}。\n可開始記錄病患個人化 ROM；馬達仍為 DRY RUN。")
        reader_running = True
        reader_thread = threading.Thread(target=reader_loop, daemon=True)
        reader_thread.start()
    except Exception as e:
        with state_lock:
            initializing = False
            initialized = False
        set_status(f"初始化或校正失敗: {e}")

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/api/init", methods=["POST"])
def api_init():
    global initializing, reader_running
    params = request.get_json(force=True) or {}
    with state_lock:
        if initializing:
            return jsonify({"ok": False, "message": "系統正在初始化，請稍等。"})
        if sensor is not None:
            sensor.motor_enabled = False
        rom_calibration.reset()
        initializing = True
    reader_running = False
    stop_motor_bridge_safely()
    threading.Thread(target=init_worker, args=(params,), daemon=True).start()
    return jsonify({"ok": True, "message": "已開始初始化與 5 秒校正。"})

@app.route("/api/status")
def api_status():
    with state_lock:
        return jsonify({
            "status": system_status,
            "initialized": initialized,
            "initializing": initializing,
            "recording": recording,
            "record_count": record_count,
            "csv_path": current_csv_path,
            "motor_bridge_connected": motor_bridge_connected,
            "motor_bridge_error": motor_bridge_error,
            "motor_bridge_last_line": motor_bridge.last_line if motor_bridge is not None else "",
            "motor_bridge_last_sent": motor_bridge.last_sent_line if motor_bridge is not None and hasattr(motor_bridge, "last_sent_line") else "",
            "motor_output": {
                "mode": "dry_run" if not LIVE_MOTOR_OUTPUT_ALLOWED else "live",
                "control_profile": MOTOR_CONTROL_PROFILE,
                "pwm_limit": MOTOR_PWM_LIMIT,
                "antagonist_release_gain": 0.8,
                "command_filter_tau": 0.18,
                "wind_slew_rate": 120.0,
                "release_slew_rate": 160.0,
                "reverse_deadtime": 0.15,
                "shoulder_release_max_pwm": 60.0,
                "live_output_allowed": LIVE_MOTOR_OUTPUT_ALLOWED,
                "encoder_safety_enabled": False,
                "lock_reason": (
                    "Motor wiring, direction, encoder/limit inputs and physical emergency stop are not verified."
                    if not LIVE_MOTOR_OUTPUT_ALLOWED else ""
                ),
            },
            "rom_calibration": rom_calibration.status(),
            "frame": serialize_frame(latest_frame),
        })

@app.route("/api/rom/start", methods=["POST"])
def api_rom_start():
    with state_lock:
        if sensor is None or not initialized:
            return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
        sensor.motor_enabled = False
        sensor.emergency_stop = False
        rom_calibration.start()
    stop_motor_bridge_safely()
    set_status("病患個人化 ROM 記錄已開始。實際馬達輸出仍維持 DRY RUN。")
    return jsonify({"ok": True, "message": "ROM 已重設，即將從手臂下垂的最大肘屈曲開始。", "rom_calibration": rom_calibration.status()})

@app.route("/api/rom/start_stage", methods=["POST"])
def api_rom_start_stage():
    data = request.get_json(silent=True) or {}
    try:
        with state_lock:
            if sensor is None or not initialized:
                return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
            sensor.motor_enabled = False
            status = rom_calibration.start_stage(data.get("stage"))
        stop_motor_bridge_safely()
        return jsonify({"ok": True, "message": f"已開始記錄：{status['current_stage_label']}。", "rom_calibration": status})
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/rom/finish_stage", methods=["POST"])
def api_rom_finish_stage():
    try:
        status = rom_calibration.finish_stage()
        completed = list(status.get("results", {}).keys())[-1]
        return jsonify({
            "ok": True,
            "message": f"已記錄：{ROMCalibrationSession.STAGE_LABELS[completed]}",
            "rom_calibration": status,
        })
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/rom/apply", methods=["POST"])
def api_rom_apply():
    try:
        with state_lock:
            if sensor is None or not initialized:
                return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
            mapping = rom_calibration.apply()
            sensor.apply_rom_calibration(
                elbow_axis=mapping["elbow_axis"],
                elbow_sign=mapping["elbow_sign"],
                front_reference=mapping["front_reference"],
                side_reference=mapping["side_reference"],
                rom=mapping["rom"],
            )
            sensor.trajectory_min_angle = mapping["rom"]["elbow_extension_target_deg"]
            sensor.trajectory_max_angle = mapping["rom"]["elbow_flexion_target_deg"]
            sensor.fixed_target_angle = mapping["rom"]["elbow_flexion_target_deg"]
        return jsonify({
            "ok": True,
            "message": f"個人化 ROM 已套用：肘關節 {mapping['elbow_axis']} 軸、方向 {mapping['elbow_sign']:+.0f}，肘訓練範圍已設為 90%。",
            "rom": mapping["rom"],
            "rom_calibration": rom_calibration.status(),
        })
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/rom/reset", methods=["POST"])
def api_rom_reset():
    with state_lock:
        if sensor is not None:
            sensor.motor_enabled = False
        rom_calibration.reset()
    stop_motor_bridge_safely()
    return jsonify({"ok": True, "message": "病患個人化 ROM 已重設。", "rom_calibration": rom_calibration.status()})

@app.route("/api/apply_params", methods=["POST"])
def api_apply_params():
    data = request.get_json(force=True) or {}
    with state_lock:
        if sensor is None:
            return jsonify({"ok": False, "message": "尚未初始化 IMU。"})
        apply_params_to_sensor(data, reset_controller=True)
        muscle_allocator.reset_conditioner()
    return jsonify({"ok": True, "message": "目標關節、復健軌跡與控制參數已套用，控制器狀態已重置。"})

@app.route("/api/set_motor", methods=["POST"])
def api_set_motor():
    data = request.get_json(force=True) or {}
    enabled = safe_bool(data.get("enabled", False))

    with state_lock:
        if sensor is None:
            return jsonify({"ok": False, "message": "尚未初始化 IMU。"})
        if enabled and not LIVE_MOTOR_OUTPUT_ALLOWED:
            sensor.motor_enabled = False
            return jsonify({
                "ok": False,
                "message": "馬達 DRY RUN：實際 PWM 輸出仍被硬鎖定；目前只顯示三路 Desired PWM，ESP32 只接收 STOP。"
            }), 403
    if enabled:
        ok = connect_motor_bridge_if_needed()
        if not ok:
            with state_lock:
                sensor.motor_enabled = False
            return jsonify({
                "ok": False,
                "message": f"ESP32 連線失敗：{motor_bridge_error}。請確認 USB 線、port={ESP32_PORT}、ESP32 程式已燒錄。"
            })

    with state_lock:
        sensor.motor_enabled = enabled
        if enabled:
            sensor.emergency_stop = False
            msg = "Motor enabled：PWM 會經由 USB Serial 輸出到 ESP32。"
        else:
            sensor.emergency_stop = False
            stop_motor_bridge_safely()
            msg = "Motor disabled：已送 STOP 給 ESP32。"

    return jsonify({"ok": True, "message": msg})

@app.route("/api/emergency_stop", methods=["POST"])
def api_emergency_stop():
    with state_lock:
        if sensor is not None:
            sensor.emergency_stop = True
            sensor.motor_enabled = False
    stop_motor_bridge_safely()
    return jsonify({"ok": True, "message": "Emergency stop 已啟動：已送 STOP 給 ESP32。"})

@app.route("/api/clear_emergency", methods=["POST"])
def api_clear_emergency():
    with state_lock:
        if sensor is not None:
            sensor.emergency_stop = False
    return jsonify({"ok": True, "message": "急停已解除。若要輸出控制命令，請重新按 Enable Motor。"})

@app.route("/api/start_recording", methods=["POST"])
def api_start_recording():
    global recording, csv_file, csv_writer, current_csv_path, record_count, participant_id, session_name, output_dir, current_label
    data = request.get_json(force=True) or {}
    with state_lock:
        if sensor is None or not initialized:
            return jsonify({"ok": False, "message": "尚未初始化 IMU。"})
        if recording:
            return jsonify({"ok": False, "message": "已經在錄製中。"})
        participant_id = str(data.get("participant_id", "S01")).strip() or "S01"
        session_name = str(data.get("session_name", "session")).strip() or "session"
        current_label = str(data.get("label", "trial")).strip() or "trial"
        output_dir = str(data.get("output_dir", "control_data")).strip() or "control_data"
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        current_csv_path = os.path.abspath(os.path.join(output_dir, f"{participant_id}_{session_name}_{current_label}_{timestamp}.csv"))
        try:
            csv_file = open(current_csv_path, "w", newline="", encoding="utf-8-sig")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                "wall_time","elapsed_time","participant_id","session","trial_label",
                "target_joint","target_mode","controller_mode",
                "target_angle","measured_angle","measured_angle_raw","tracking_error",
                "elbow_angle","elbow_angle_raw","shoulder_angle","shoulder_angle_raw",
                "error_for_control","integral_error","derivative_error","filtered_measurement_velocity","control_active","anti_windup_active",
                "raw_pid_output","pid_output","motor_cmd","servo_cmd",
                "desired_motor_cmd","desired_pwm_biceps","desired_pwm_triceps","desired_pwm_deltoid",
        "pwm_biceps","pwm_triceps","pwm_deltoid",
                "encoder_count_biceps","encoder_count_triceps","encoder_count_deltoid",
                "encoder_vel_biceps","encoder_vel_triceps","encoder_vel_deltoid",
                "upper_angle","forearm_angle","elbow_velocity","elbow_acceleration","jerk",
                "shoulder_motion_state","shoulder_release_command","shoulder_release_pwm","shoulder_motor_enable",
                "motor_enabled","emergency_stop","motion_state"
            ])
            record_count = 0
            recording = True
        except Exception as e:
            return jsonify({"ok": False, "message": f"建立 CSV 失敗: {e}"})
    return jsonify({"ok": True, "message": "開始錄製。", "csv_path": current_csv_path})

@app.route("/api/stop_recording", methods=["POST"])
def api_stop_recording():
    global recording, csv_file, csv_writer
    with state_lock:
        if not recording:
            return jsonify({"ok": True, "message": "目前沒有在錄製。"})
        recording = False
        if csv_file is not None:
            csv_file.flush(); csv_file.close()
        csv_file = None
        csv_writer = None
    return jsonify({"ok": True, "message": f"錄製完成，共 {record_count} 筆。檔案位置: {current_csv_path}"})

if __name__ == "__main__":
    print("======================================")
    print("IMU 關節軌跡控制 Web GUI 啟動")
    print("瀏覽器開啟：http://樹莓派IP:5000")
    print("本機：http://127.0.0.1:5000")
    print("======================================")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
