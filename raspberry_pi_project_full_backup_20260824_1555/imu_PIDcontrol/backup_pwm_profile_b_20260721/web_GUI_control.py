import csv
import os
import threading
import time
from datetime import datetime

from flask import Flask, jsonify, request, render_template_string

from imu_control import IMURehabSystem
from imu_validation import IMUValidationSession
from muscle_allocator import MuscleAllocator
from esp32motor import ESP32MotorBridge

app = Flask(__name__)

state_lock = threading.Lock()
sensor = None
reader_thread = None
reader_running = False
latest_frame = None
history = []
imu_validation = IMUValidationSession()

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
# Deliberate hardware commissioning lock. Keep False until Motor 1 direction,
# limit switches/encoder and an accessible physical emergency stop are verified.
LIVE_MOTOR_OUTPUT_ALLOWED = False

motor_bridge = None
motor_bridge_connected = False
motor_bridge_error = ""

muscle_allocator = MuscleAllocator(
    pwm_limit=MOTOR_PWM_LIMIT,
    active_min_pwm=0,
    antagonist_release_gain=0.35,
    shoulder_release_pwm=50,
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
    .container{display:grid;grid-template-columns:390px 1fr;gap:16px;padding:16px}.card{background:white;border-radius:12px;padding:16px;box-shadow:0 2px 8px rgba(0,0,0,.08);margin-bottom:16px}.card h2{font-size:18px;margin:0 0 12px}label{display:block;font-size:13px;color:#374151;margin:7px 0 4px}input,select{width:100%;box-sizing:border-box;padding:8px;border:1px solid #cbd5e1;border-radius:8px}button{width:100%;padding:10px;border:0;border-radius:8px;margin:5px 0;background:#2563eb;color:white;font-size:15px;cursor:pointer}button:hover{background:#1d4ed8}button:disabled{background:#9ca3af;cursor:not-allowed}.secondary{background:#64748b}.success{background:#16a34a}.danger{background:#dc2626}.warning{background:#f59e0b;color:#111827}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.status{background:#eef2ff;border-left:5px solid #4f46e5;border-radius:8px;padding:10px;white-space:pre-wrap;font-size:14px}.validation{background:#f8fafc;border:1px solid #d1d5db;border-radius:8px;padding:10px;margin:8px 0;white-space:pre-wrap;font-size:13px;line-height:1.5}.pass{color:#15803d;font-weight:700}.fail{color:#b91c1c;font-weight:700}.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}.metric{background:#f8fafc;border:1px solid #e5e7eb;border-radius:10px;padding:12px}.metric-title{font-size:13px;color:#64748b}.metric-value{font-size:22px;font-weight:700;margin-top:6px}.small{font-size:13px;color:#64748b;line-height:1.45}.rec{color:#dc2626;font-weight:700}.idle{color:#64748b;font-weight:700}canvas{width:100%;height:430px;background:white;border:1px solid #e5e7eb;border-radius:10px}@media(max-width:900px){.container{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(2,1fr)}}
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
      <h2>2. IMU 安裝與方向驗證</h2>
      <p class="small">快速開機檢查約 42 秒：靜止 6 秒，再各做 2 次肘屈伸、肩前舉與肩側舉。只驗證穩定度、方向與回零，不代表絕對角度準確度。</p>
      <button onclick="startValidation()">開始／重新開始 42 秒快速驗證</button>
      <button id="validationStageButton" class="secondary" onclick="startValidationStage()" disabled>開始目前階段</button>
      <button id="validationApplyButton" class="success" onclick="applyValidation()" disabled>確認並套用結果</button>
      <div class="validation" id="validationGuide">請先初始化並校正 IMU。</div>
      <div class="validation" id="validationResults">尚無驗證結果。</div>
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
      <div class="validation fail" id="dryRunBanner">馬達 DRY RUN：可顯示三路 Desired PWM，實際 Output 固定為 0，ESP32 只接收 STOP。</div>
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

// 顯示視窗設定：目前時間會畫在圖中間偏左，右側保留未來目標軌跡給使用者預看。
const maxPoints = 1200;
const pastWindowSeconds = 8;
const futureWindowSeconds = 8;

function setText(id, text){document.getElementById(id).innerText=text;}
async function postJSON(url,data){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data||{})});return await r.json();}
async function getJSON(url){const r=await fetch(url);return await r.json();}
function fval(id, def){const v=parseFloat(document.getElementById(id).value); return Number.isFinite(v)?v:def;}

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
  const res=await postJSON('/api/init',params());
  setText('statusBox',res.message);
}
async function startValidation(){
  const res=await postJSON('/api/validation/start',{});
  if(!res.ok){alert(res.message);return;}
  setText('statusBox',res.message);
  validationFinishing=false;
  validationAutoRun=true;
  await refreshStatus();
  await startValidationStage();
}
async function startValidationStage(){
  if(validationStageStarting)return;
  validationStageStarting=true;
  const res=await postJSON('/api/validation/start_stage',{});
  validationStageStarting=false;
  if(!res.ok){validationAutoRun=false;alert(res.message);return;}
  validationFinishing=false;
  setText('statusBox',res.message);
  await refreshStatus();
}
async function finishValidationStage(){
  if(validationFinishing)return;
  validationFinishing=true;
  const res=await postJSON('/api/validation/finish_stage',{});
  if(!res.ok)alert(res.message);
  else setText('statusBox',res.message);
  await refreshStatus();
  validationFinishing=false;
  if(res.ok && validationAutoRun && res.validation && res.validation.expected_stage){
    setTimeout(startValidationStage,400);
  }else if(res.ok){
    validationAutoRun=false;
  }
}
async function applyValidation(){
  const res=await postJSON('/api/validation/apply',{});
  if(!res.ok){alert(res.message);return;}
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
  updateValidationPanel(data.validation||{});
  if(data.motor_output){
    const s=data.motor_output;
    setText('dryRunBanner','馬達 '+String(s.mode).toUpperCase()+'：Desired PWM 上限 ±'+s.pwm_limit
      +'；實際輸出 '+(s.live_output_allowed?'已允許':'固定為 0，ESP32 只接收 STOP')+'。');
    setText('motorInterlockText',s.live_output_allowed?'硬體輸出鎖已解除。':('硬體輸出鎖未解除：'+s.lock_reason));
  }
  if(data.recording){setText('recordingState','錄製中，筆數：'+data.record_count);document.getElementById('recordingState').className='rec';setText('csvPathText',data.csv_path||'');}
  if(data.frame){
    const f=data.frame;
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
  const interlock=v.motor_interlock_reason||'IMU 驗證已通過並套用。';
  setText('motorInterlockText',interlock);

  if(!v.state || v.state==='idle'){
    setText('validationGuide','請先初始化並校正 IMU，再按「開始驗證」。');
    setText('validationResults','尚無驗證結果。');
    stageButton.disabled=true;applyButton.disabled=true;return;
  }

  if(v.current_stage){
    const guide=v.current_stage_label+'\n'+(v.phase||'')+'\n剩餘 '+fmtMetric(v.remaining,1)+' 秒｜樣本 '+(v.sample_count||0);
    setText('validationGuide',guide);
    stageButton.disabled=true;
    if(v.remaining<=0.05)finishValidationStage();
  }else if(v.expected_stage){
    setText('validationGuide','下一階段：'+v.expected_stage_label+'\n確認姿勢與空間安全後，按「開始目前階段」。');
    stageButton.disabled=false;
  }else if(v.can_apply && !v.applied){
    setText('validationGuide','全部測試通過。請檢查建議軸向後，按「確認並套用結果」。');
    stageButton.disabled=true;
  }else if(v.applied){
    setText('validationGuide','IMU 驗證已套用，本次初始化有效。');
    stageButton.disabled=true;
  }else{
    setText('validationGuide','驗證未通過。請調整 IMU 固定方式後重新開始。');
    stageButton.disabled=true;
  }
  applyButton.disabled=!v.can_apply||v.applied;

  const labels={static:'靜止',elbow:'肘屈伸',shoulder_front:'前平舉',shoulder_side:'側平舉'};
  let lines=[];
  const results=v.results||{};
  Object.keys(labels).forEach(key=>{
    const r=results[key]; if(!r)return;
    lines.push((r.passed?'PASS ':'FAIL ')+labels[key]);
    if(key==='static' && r.metrics){
      const e=r.metrics.elbow_roll||{}, s=r.metrics.shoulder||{};
      lines.push('  roll std/drift='+fmtMetric(e.std)+'°/'+fmtMetric(e.drift)+'°｜肩 std/drift='+fmtMetric(s.std)+'°/'+fmtMetric(s.drift)+'°');
    }
    if(key==='elbow'){
      const m=(r.candidates||{})[r.selected_axis]||{};
      lines.push('  軸='+r.selected_axis+' sign='+r.selected_sign+'｜方向='+fmtMetric((m.direction_agreement||0)*100,0)+'%｜回零最大='+fmtMetric(m.max_return_error)+'°');
    }
    if(key.indexOf('shoulder_')===0 && r.metrics){
      lines.push('  方向='+fmtMetric((r.metrics.direction_agreement||0)*100,0)+'%｜回零最大='+fmtMetric(r.metrics.max_return_error)+'°｜平面分類='+fmtMetric((r.plane_accuracy||0)*100,0)+'%');
    }
    (r.reasons||[]).forEach(reason=>lines.push('  - '+reason));
  });
  if(v.recommendation){
    const r=v.recommendation;
    lines.push('建議：elbow axis='+r.elbow_axis+', sign='+r.elbow_sign+', 信心度='+fmtMetric(r.confidence_percent,1)+'%');
    if(r.shoulder_plane_separation_deg!==undefined) lines.push('肩平面分離角='+fmtMetric(r.shoulder_plane_separation_deg,1)+'°');
  }
  (v.failure_reasons||[]).forEach(reason=>lines.push('失敗原因：'+reason));
  setText('validationResults',lines.length?lines.join('\n'):'等待第一階段資料。');
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
        "omega","alpha","jerk","shoulder_angle","shoulder_angle_raw","shoulder_angle_velocity",
        "shoulder_dx","shoulder_dy","shoulder_dz","target_angle","error","error_for_control","error_active_time",
        "integral_error","derivative_error","raw_pid_output","pid_output","motor_cmd","servo_cmd",
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
    for k in ["shoulder_assist","shoulder_release_command","shoulder_motor_enable","shoulder_accel_valid","motor_enabled","emergency_stop"]:
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
            imu_validation.add_sample(frame)

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
                shoulder_motor_enable=frame.get("shoulder_motor_enable", True),
                feedback=fb,
            )

            output_pwm = muscle_allocator.allocate(
                target_joint=frame.get("target_joint", "elbow"),
                motor_cmd=frame.get("motor_cmd", 0),
                control_profile=MOTOR_CONTROL_PROFILE,
                motor_enabled=frame.get("motor_enabled", False) and LIVE_MOTOR_OUTPUT_ALLOWED,
                emergency_stop=frame.get("emergency_stop", False),
                shoulder_release_command=frame.get("shoulder_release_command", False),
                shoulder_motor_enable=frame.get("shoulder_motor_enable", True),
                feedback=fb,
            )

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
                        frame.get("raw_pid_output"), frame.get("pid_output"), frame.get("motor_cmd"), frame.get("servo_cmd"),
                        frame.get("desired_motor_cmd"), frame.get("desired_pwm_biceps"), frame.get("desired_pwm_triceps"), frame.get("desired_pwm_deltoid"),
                        frame.get("pwm_biceps"), frame.get("pwm_triceps"), frame.get("pwm_deltoid"),
                        frame.get("encoder_count_biceps"), frame.get("encoder_count_triceps"), frame.get("encoder_count_deltoid"),
                        frame.get("encoder_vel_biceps"), frame.get("encoder_vel_triceps"), frame.get("encoder_vel_deltoid"),
                        frame.get("upper_angle"), frame.get("forearm_angle"), frame.get("omega"), frame.get("alpha"), frame.get("jerk"),
                        frame.get("shoulder_motion_state"), frame.get("shoulder_release_command"), frame.get("shoulder_motor_enable"),
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
        with state_lock:
            sensor = new_sensor
            imu_validation.reset()
            initialized = True
            initializing = False
        if ENABLE_AUTO_CONNECT_ESP32:
            ok = connect_motor_bridge_if_needed()
            if ok:
                set_status(f"校正完成，樣本數: {result['sample_count']}。\nESP32 motor bridge 已連線。\n請先完成 IMU 驗證；馬達目前鎖定。")
            else:
                set_status(f"校正完成，樣本數: {result['sample_count']}。\nESP32 尚未連線：{motor_bridge_error}\n請先完成 IMU 驗證；馬達目前鎖定。")
        else:
            set_status(f"校正完成，樣本數: {result['sample_count']}。\n請先完成 IMU 穩定度與方向驗證；馬達目前鎖定。")
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
        imu_validation.reset()
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
                "live_output_allowed": LIVE_MOTOR_OUTPUT_ALLOWED,
                "encoder_safety_enabled": False,
                "lock_reason": (
                    "Motor wiring, direction, encoder/limit inputs and physical emergency stop are not verified."
                    if not LIVE_MOTOR_OUTPUT_ALLOWED else ""
                ),
            },
            "validation": imu_validation.status(),
            "frame": serialize_frame(latest_frame),
        })

@app.route("/api/validation/start", methods=["POST"])
def api_validation_start():
    with state_lock:
        if sensor is None or not initialized:
            return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
        sensor.motor_enabled = False
        sensor.emergency_stop = False
        imu_validation.start()
    stop_motor_bridge_safely()
    set_status("IMU 驗證已開始。驗證期間馬達維持停用。")
    return jsonify({"ok": True, "message": "驗證已重設，請開始靜止穩定度測試。", "validation": imu_validation.status()})

@app.route("/api/validation/start_stage", methods=["POST"])
def api_validation_start_stage():
    data = request.get_json(silent=True) or {}
    try:
        with state_lock:
            if sensor is None or not initialized:
                return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
            sensor.motor_enabled = False
            status = imu_validation.start_stage(data.get("stage"))
        stop_motor_bridge_safely()
        return jsonify({"ok": True, "message": f"已開始：{status['current_stage_label']}。馬達維持停用。", "validation": status})
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/validation/finish_stage", methods=["POST"])
def api_validation_finish_stage():
    try:
        status = imu_validation.finish_stage()
        completed = list(status.get("results", {}).keys())[-1]
        result = status["results"][completed]
        verdict = "PASS" if result.get("passed") else "FAIL"
        return jsonify({
            "ok": True,
            "message": f"{IMUValidationSession.STAGE_LABELS[completed]}：{verdict}",
            "validation": status,
        })
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/validation/apply", methods=["POST"])
def api_validation_apply():
    try:
        with state_lock:
            if sensor is None or not initialized:
                return jsonify({"ok": False, "message": "請先初始化並校正 IMU。"}), 400
            mapping = imu_validation.apply()
            sensor.apply_imu_validation_mapping(**mapping)
        return jsonify({
            "ok": True,
            "message": f"IMU 驗證已套用：肘關節 {mapping['elbow_axis']} 軸，方向符號 {mapping['elbow_sign']:+.0f}。",
            "validation": imu_validation.status(),
        })
    except ValueError as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400

@app.route("/api/validation/reset", methods=["POST"])
def api_validation_reset():
    with state_lock:
        if sensor is not None:
            sensor.motor_enabled = False
        imu_validation.reset()
    stop_motor_bridge_safely()
    return jsonify({"ok": True, "message": "IMU 驗證已重設，馬達已鎖定。", "validation": imu_validation.status()})

@app.route("/api/apply_params", methods=["POST"])
def api_apply_params():
    data = request.get_json(force=True) or {}
    with state_lock:
        if sensor is None:
            return jsonify({"ok": False, "message": "尚未初始化 IMU。"})
        apply_params_to_sensor(data, reset_controller=True)
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
        if enabled and not imu_validation.applied:
            sensor.motor_enabled = False
            return jsonify({
                "ok": False,
                "message": "馬達已鎖定：本次初始化尚未完成並套用 IMU 驗證。"
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
                "error_for_control","integral_error","derivative_error",
                "raw_pid_output","pid_output","motor_cmd","servo_cmd",
                "desired_motor_cmd","desired_pwm_biceps","desired_pwm_triceps","desired_pwm_deltoid",
        "pwm_biceps","pwm_triceps","pwm_deltoid",
                "encoder_count_biceps","encoder_count_triceps","encoder_count_deltoid",
                "encoder_vel_biceps","encoder_vel_triceps","encoder_vel_deltoid",
                "upper_angle","forearm_angle","elbow_velocity","elbow_acceleration","jerk",
                "shoulder_motion_state","shoulder_release_command","shoulder_motor_enable",
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
