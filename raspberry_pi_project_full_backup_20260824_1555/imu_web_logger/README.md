# Raspberry Pi IMU Monitor

一個用於 Raspberry Pi 的即時 IMU 監控系統，支援 MPU6050 感測器。透過網頁介面即時顯示加速度、陀螺儀和角度資料，並支援 CSV 資料記錄功能。

## 功能特性

- **即時 IMU 監控**：實時讀取 MPU6050 的加速度計和陀螺儀資料（10 Hz 更新頻率）
- **網頁介面**：美觀的響應式網頁，可在筆電瀏覽器中查看數據
- **實時波形圖表**：使用 Chart.js 繪製 Roll、Pitch、Yaw 的即時折線圖
- **角度計算**：透過加速度估算 Roll 和 Pitch 角度
- **CSV 記錄**：支援帶標籤的 CSV 檔案記錄，用於訓練資料集
- **錯誤處理**：硬體連接失敗時不會導致程式崩潰

## 硬體需求

- Raspberry Pi（任何版本，建議 Pi 3 或以上）
- MPU6050 IMU 感測器
- I2C 連接線

## 接線方式

```
MPU6050 → Raspberry Pi
GND     → GND
VCC     → 3.3V
SDA     → GPIO 2 (Pin 3)
SCL     → GPIO 3 (Pin 5)
```

## 環境設置

### 1. 啟用 I2C

編輯 Raspberry Pi 配置：

```bash
sudo raspi-config
```

進入 `Interfacing Options` → `I2C` → Enable

### 2. 檢查 MPU6050 連接

使用 `i2cdetect` 驗證硬體：

```bash
sudo apt-get install i2c-tools
i2cdetect -y 1
```

如果看到 `68`，表示 MPU6050 已正確連接：

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

### 3. 安裝 Python 套件

```bash
pip install -r requirements.txt
```

如果遇到 `smbus2` 安裝問題，可嘗試：

```bash
sudo apt-get install python3-dev
pip install smbus2
```

## 專案結構

```
imu_web_logger/
├── app.py              # Flask 應用程式（包含 HTML、CSS、JavaScript）
├── requirements.txt    # Python 依賴
├── data/              # 錄製的 CSV 檔案存放位置
└── README.md          # 本檔案
```

## 啟動應用

### 在 Raspberry Pi 上

```bash
cd imu_web_logger
python app.py
```

輸出應如下：

```
============================================================
Raspberry Pi IMU Monitor - Starting Up
============================================================
✓ MPU6050 initialized at address 0x68
► Data reader thread started

✓ Flask server starting on http://0.0.0.0:5000
✓ Press Ctrl+C to stop
```

### 在筆電上訪問

1. 查詢 Raspberry Pi IP：

```bash
# 在 Raspberry Pi 上執行
hostname -I
```

2. 在筆電瀏覽器打開：

```
http://[樹莓派IP]:5000
```

例如：`http://192.168.1.100:5000`

## 使用說明

### 監控面板

- **Roll / Pitch / Yaw**：實時角度顯示（度數）
- **Raw IMU Data**：原始加速度和陀螺儀數值
- **Real-time Waveform**：過去 100 個數據點的折線圖

### CSV 記錄

1. 選擇動作標籤（Rest / Flexion / Extension / Hold / Unknown）
2. 點擊 **Start Recording** 開始記錄
3. 感應器會開始記錄所有 IMU 資料到 CSV 檔案
4. 點擊 **Stop Recording** 停止記錄

錄製的 CSV 檔案會存放在 `data/` 資料夾，檔名格式為：

```
imu_YYYYMMDD_HHMMSS_label.csv
```

例如：`imu_20240524_145320_flexion.csv`

### CSV 檔案內容

```csv
time,ax,ay,az,gx,gy,gz,roll,pitch,yaw,label
0.000,0.1234,-0.5678,9.8765,1.23,-2.34,5.67,10.5,-5.3,5.67,flexion
0.100,0.1245,-0.5690,9.8750,1.25,-2.36,5.69,10.6,-5.4,5.69,flexion
...
```

- **time**：從開始錄製後經過的秒數（精度到毫秒）
- **ax, ay, az**：加速度（g 單位）
- **gx, gy, gz**：陀螺儀角速度（°/s）
- **roll, pitch, yaw**：計算出的角度（度數）
- **label**：使用者選擇的標籤

## 角度計算方式

使用加速度計估算角度（補充濾波器的第一步）：

```
roll = atan2(ay, az) × 180 / π
pitch = atan2(-ax, √(ay² + az²)) × 180 / π
```

- 不使用磁力計，所以 **yaw 不穩定**（會漂移）
- roll 和 pitch 適合用於**肘關節角度回授**
- 陀螺儀數據可用於短期精確度量

## 進階設置

### 調整更新頻率

在 `app.py` 中修改：

```python
DATA_UPDATE_INTERVAL = 0.1  # 改為 0.05 表示 20 Hz
```

### 調整圖表點數

```python
MAX_DATA_POINTS = 100  # 改為 200 保留更多歷史資料
```

### 修改 I2C 匯流排

如果使用其他 I2C 匯流排（預設是 1）：

```python
I2C_BUS = 0  # 改為 0
```

## 故障排除

### 問題：Unable to locate package mpu6050-raspberrypi

如果 pip 找不到 `mpu6050-raspberrypi`，可使用替代方案：

```bash
pip install mpu6050-api
```

或使用 smbus2 直接操作：

程式會自動偵測，如果 mpu6050 庫不可用，會在模擬模式下執行。

### 問題：Connection refused (拒絕連接)

1. 確保 Flask 伺服器已啟動
2. 檢查防火牆設置
3. 確認 Raspberry Pi 和筆電在同一網路

### 問題：I2C 找不到 0x68

1. 檢查接線是否正確
2. 確認 I2C 已啟用：`sudo raspi-config`
3. 檢查 MPU6050 是否已接電

### 問題：角度波動太大

- 可在 `app.py` 中添加移動平均濾波（Moving Average）
- 或實現互補濾波器（Complementary Filter）
- 目前版本以「即時波形顯示」和「CSV 訓練資料記錄」為主

## 相關資源

- [MPU6050 Datasheet](https://store.invensense.com/datasheets/invensense/mpu6050_datasheet.pdf)
- [Flask 文檔](https://flask.palletsprojects.com/)
- [Chart.js 文檔](https://www.chartjs.org/)

## 注意事項

⚠️ **yaw 值不穩定**
- MPU6050 沒有磁力計，yaw 值會漂移
- 適合用於短期監測，不能作為長期穩定的角度參考

💡 **改進建議**
- 後續可添加卡爾曼濾波器提高準確度
- 添加磁力計（HMC5883L）以獲得穩定的 yaw 值
- 實現補充濾波器結合加速度和陀螺儀數據

## 許可證

MIT License

## 作者

Raspberry Pi IMU Monitor Project
