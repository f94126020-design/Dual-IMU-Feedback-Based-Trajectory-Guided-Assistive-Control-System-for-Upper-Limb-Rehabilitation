# ESP32 Motor Safety Node

## 已驗證的編譯環境

- Arduino IDE 2.x
- ESP32 by Espressif Systems：3.3.10
- Board：ESP32 Dev Module
- Baud rate：115200
- 編譯結果：Flash 22%、RAM 6%

## 重要安全條件

第一次燒錄與通訊測試時：

1. ESP32 可以接 USB。
2. BTS7960 與馬達電源保持關閉，或先完全不接馬達。
3. 不要連接病患、繩索或外骨骼。
4. 樹莓派的 `LIVE_MOTOR_OUTPUT_ALLOWED` 必須繼續保持 `False`。

## Arduino IDE 燒錄

1. 使用全英文路徑開啟：
   `C:\Users\USER\Desktop\ESP32_Motor_Safety_Node\ESP32_Motor_Safety_Node.ino`
2. Tools → Board → esp32 → ESP32 Dev Module。
3. 選擇 ESP32 對應的 COM Port。
4. Upload Speed 先使用 921600；若失敗改成 115200。
5. 按 Verify，確認編譯成功。
6. 按 Upload。
7. 開啟 Serial Monitor，設定 115200 baud 與 New Line。

## 開機預期訊息

```text
READY,ESP32_MOTOR_SAFETY_NODE,V1
STATE,IDLE,-1,0,0,0,0,0,0
```

此時三顆驅動器的 REN／LEN 應維持 LOW。

## 不接馬達的通訊測試

### 1. 未 ARM 直接送 PWM

輸入：

```text
PWM,20,-16,0
```

預期：

```text
ERR,NOT_ARMED
```

### 2. ARM

輸入：

```text
ARM
```

預期：

```text
OK,ARM,ARMED
```

若 300 ms 內沒有收到有效 PWM，預期自動進入：

```text
FAULT,COMMAND_TIMEOUT
STATE,TIMEOUT_STOP,...
```

### 3. 清除 Timeout

輸入：

```text
CLEAR
```

預期：

```text
OK,CLEAR,IDLE
```

### 4. ESTOP 鎖存

輸入：

```text
ESTOP
PWM,20,-16,0
```

預期包含：

```text
FAULT,EMERGENCY_STOP
OK,ESTOP
ERR,ESTOP_LATCHED
```

### 5. 解除 ESTOP

依序輸入：

```text
CLEAR
ARM
```

預期回到 ARMED；仍需持續收到有效 PWM 才不會進入 Timeout。

## 樹莓派整合前仍需完成

- Pi 控制迴圈改成固定 50 Hz。
- Serial 寫入加入 lock。
- 連線後先送 `ARM`，確認 `OK,ARM,ARMED` 才允許 PWM。
- 解析 `STATE,...`。
- Timeout／ESTOP 時同步鎖住 Pi 的 motor enable。
- 完成以上測試前，不得將 `LIVE_MOTOR_OUTPUT_ALLOWED` 改成 `True`。
