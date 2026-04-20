#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BME680.h>
#include <Adafruit_Sensor.h>
#include <math.h>

/* ================= WIFI CONFIG ================= */
const char* ssid     = "RUDRA_14";
const char* password = "Rudra@wifi123";

/* ================= WEB SERVER ================== */
WebServer server(80);

/* ================= I2C PINS (ESP32) ============ */
#define I2C_SDA 21
#define I2C_SCL 22

/* ================= SENSORS ===================== */
Adafruit_MPU6050 mpu;
Adafruit_BMP3XX bmp388;
Adafruit_BME680 bme680;

/* ================= STATE VARIABLES ============= */
// Attitude
float roll  = 0.0f;    // rad
float pitch = 0.0f;    // rad
float yaw   = 0.0f;    // rad 

// Velocities (Dead Reckoning)
float vx_est = 0.0f;   // m/s (World X)
float vy_est = 0.0f;   // m/s (World Y)
const float VEL_DAMPING = 0.99f; // Leaky integrator to prevent infinite drift

// Z-Axis Fusion (Kalman)
float z_est = 0.0f;    // fused altitude (m)
float vz_est = 0.0f;   // fused vertical speed (m/s)

float P00 = 0.0f, P01 = 0.0f, P10 = 0.0f, P11 = 0.0f;
float Q_pos = 0.01f, Q_vel = 0.15f, R_alt = 0.1f; // Tighter R for BMP388

// Air Data
float air_temp = 0.0f; // Celsius
float air_hum  = 0.0f; // %
float air_gas  = 0.0f; // KOhms

const float G = 9.81f;         
unsigned long lastMicros = 0;  

// Baro baseline
bool  baroCalibrated = false;
float z_baro0      = 0.0f;     
float z_baro_filt  = 0.0f;     
const float baroAlpha = 0.90f;

// Gyro bias
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

// Accel low-pass
float ax_f = 0.0f, ay_f = 0.0f, az_f = 0.0f;
const float accelAlpha = 0.4f;

// Timing
unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_INTERVAL_MS = 50; // 20 Hz IMU

unsigned long lastAirDataUpdate = 0;
const unsigned long AIRDATA_INTERVAL_MS = 1000; // 1 Hz BME680 (prevent blocking)

/* ============ STATIC HTML PAGE ================= */
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ESP32 ADAHRS Telemetry</title>
  <style>
    body { font-family: Arial, sans-serif; background: #eef2f5; }
    .container { display: flex; flex-wrap: wrap; max-width: 800px; margin: 20px auto; gap: 20px; }
    .card { background: #fff; padding: 20px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); flex: 1; min-width: 300px; }
    h2 { margin-top: 0; color: #333; border-bottom: 2px solid #ddd; padding-bottom: 5px; }
    .data-row { display: flex; justify-content: space-between; margin-bottom: 8px; }
    .label { color: #666; font-weight: bold; }
    .value { font-family: monospace; font-size: 18px; color: #0056b3; }
  </style>
</head>
<body>
  <h1 style="text-align:center; color:#333;">ADAHRS Flight Dashboard</h1>
  <div class="container">
    <div class="card">
      <h2>Attitude & Heading</h2>
      <div class="data-row"><span class="label">Roll (deg)</span><span id="roll" class="value">--</span></div>
      <div class="data-row"><span class="label">Pitch (deg)</span><span id="pitch" class="value">--</span></div>
      <div class="data-row"><span class="label">Yaw (deg)</span><span id="yaw" class="value">--</span></div>
    </div>
    <div class="card">
      <h2>Kinematics & Altimetry</h2>
      <div class="data-row"><span class="label">Altitude (m)</span><span id="alt" class="value">--</span></div>
      <div class="data-row"><span class="label">Vel Z (m/s)</span><span id="vz" class="value">--</span></div>
      <div class="data-row"><span class="label">Vel X (m/s)</span><span id="vx" class="value">--</span></div>
      <div class="data-row"><span class="label">Vel Y (m/s)</span><span id="vy" class="value">--</span></div>
    </div>
    <div class="card">
      <h2>Air Data (Environment)</h2>
      <div class="data-row"><span class="label">Temperature (°C)</span><span id="temp" class="value">--</span></div>
      <div class="data-row"><span class="label">Humidity (%)</span><span id="hum" class="value">--</span></div>
      <div class="data-row"><span class="label">Gas Res. (KΩ)</span><span id="gas" class="value">--</span></div>
    </div>
  </div>
  <script>
    function updateData() {
      fetch('/data').then(r => r.json()).then(d => {
        document.getElementById('roll').innerText = d.roll.toFixed(1);
        document.getElementById('pitch').innerText = d.pitch.toFixed(1);
        document.getElementById('yaw').innerText = d.yaw.toFixed(1);
        document.getElementById('alt').innerText = d.alt.toFixed(2);
        document.getElementById('vz').innerText = d.vz.toFixed(2);
        document.getElementById('vx').innerText = d.vx.toFixed(2);
        document.getElementById('vy').innerText = d.vy.toFixed(2);
        document.getElementById('temp').innerText = d.temp.toFixed(1);
        document.getElementById('hum').innerText = d.hum.toFixed(1);
        document.getElementById('gas').innerText = (d.gas / 1000).toFixed(1);
      });
    }
    setInterval(updateData, 500); // 2Hz refresh
  </script>
</body>
</html>
)rawliteral";

/* ============ KALMAN FILTER ==================== */
void kalmanPredict(float a_z, float dt) {
  z_est += vz_est * dt + 0.5f * a_z * dt * dt;
  vz_est += a_z * dt;

  float P00_old = P00, P01_old = P01, P10_old = P10, P11_old = P11;
  P00 = P00_old + dt * (P10_old + P01_old) + dt * dt * P11_old + Q_pos;
  P01 = P01_old + dt * P11_old;
  P10 = P10_old + dt * P11_old;
  P11 = P11_old + Q_vel;
}

void kalmanUpdate(float z_meas) {
  float y = z_meas - z_est;      
  float S = P00 + R_alt;         
  float K0 = P00 / S;
  float K1 = P10 / S;

  z_est += K0 * y;
  vz_est += K1 * y;

  float P00_old = P00, P01_old = P01;
  P00 = (1.0f - K0) * P00_old;
  P01 = (1.0f - K0) * P01_old;
}

/* ============ 3D KINEMATICS & ATTITUDE ========= */
float processKinematics(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
  // LPF Accel
  ax_f = accelAlpha * ax_f + (1.0f - accelAlpha) * ax;
  ay_f = accelAlpha * ay_f + (1.0f - accelAlpha) * ay;
  az_f = accelAlpha * az_f + (1.0f - accelAlpha) * az;

  // Gyro integration
  roll  += (gx - gyroBiasX) * dt;
  pitch += (gy - gyroBiasY) * dt;
  yaw   += (gz - gyroBiasZ) * dt; 

  // Accel-based angles
  float acc_roll  = atan2(ay_f, az_f);
  float acc_pitch = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f));

  // Complementary filter
  const float alpha = 0.985f;
  roll  = alpha * roll  + (1.0f - alpha) * acc_roll;
  pitch = alpha * pitch + (1.0f - alpha) * acc_pitch;

  // Body to World Rotation Matrix (Z-Up)
  float cph = cos(roll), sph = sin(roll);
  float cth = cos(pitch), sth = sin(pitch);
  float cps = cos(yaw), sps = sin(yaw);

  float a_x_world = (cth * cps) * ax_f + (sph * sth * cps - cph * sps) * ay_f + (cph * sth * cps + sph * sps) * az_f;
  float a_y_world = (cth * sps) * ax_f + (sph * sth * sps + cph * cps) * ay_f + (cph * sth * sps - sph * cps) * az_f;
  float a_z_world = (-sth) * ax_f + (sph * cth) * ay_f + (cph * cth) * az_f;

  float a_z_linear = a_z_world - G;

  // Deadband for static noise
  if (fabs(a_x_world) < 0.15f) a_x_world = 0.0f;
  if (fabs(a_y_world) < 0.15f) a_y_world = 0.0f;
  if (fabs(a_z_linear) < 0.06f) a_z_linear = 0.0f;

  // Integrate X/Y Velocities (Leaky Integrator)
  vx_est = (vx_est + a_x_world * dt) * VEL_DAMPING;
  vy_est = (vy_est + a_y_world * dt) * VEL_DAMPING;

  return a_z_linear;
}

/* ============ WEB HANDLERS ===================== */
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleData() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"alt\":%.2f,\"vz\":%.2f,\"vx\":%.2f,\"vy\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"temp\":%.2f,\"hum\":%.2f,\"gas\":%.0f}",
    z_est, vz_est, vx_est, vy_est,
    roll * 180.0f / PI, pitch * 180.0f / PI, yaw * 180.0f / PI,
    air_temp, air_hum, air_gas
  );
  server.send(200, "application/json", buf);
}

/* ============ SETUP ============================ */
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize MPU6050
  if (!mpu.begin()) { Serial.println("MPU6050 Failed"); while(1); }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize BMP388 (Address 0x77 or 0x76 usually)
  if (!bmp388.begin_I2C()) { Serial.println("BMP388 Failed"); while(1); }
  bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp388.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp388.setOutputDataRate(BMP3_ODR_50_HZ);

  // Initialize BME680
  if (!bme680.begin()) { Serial.println("BME680 Failed"); while(1); }
  bme680.setTemperatureOversampling(BME680_OS_8X);
  bme680.setHumidityOversampling(BME680_OS_2X);
  bme680.setPressureOversampling(BME680_OS_NONE); // Using BMP388 for pressure
  bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);

  // Gyro Calibration
  Serial.println("Calibrating Gyro...");
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
    gyroBiasX += g.gyro.x; gyroBiasY += g.gyro.y; gyroBiasZ += g.gyro.z;
    delay(3);
  }
  gyroBiasX /= 500; gyroBiasY /= 500; gyroBiasZ /= 500;

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  lastMicros = micros();
}

/* ============ LOOP ============================= */
void loop() {
  unsigned long nowMs = millis();

  // IMU & Fusion (20 Hz)
  if (nowMs - lastSensorUpdate >= SENSOR_INTERVAL_MS) {
    lastSensorUpdate = nowMs;
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1e6f;
    if (dt <= 0.0f) dt = 1e-3f;
    if (dt > 0.05f) dt = 0.05f;
    lastMicros = now;

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float a_z = processKinematics(a.acceleration.x, a.acceleration.y, a.acceleration.z,
                                  g.gyro.x, g.gyro.y, g.gyro.z, dt);

    // Read BMP388
    if (!bmp388.performReading()) return;
    float z_baro = bmp388.readAltitude(1013.25);

    if (!baroCalibrated) { z_baro0 = z_baro; baroCalibrated = true; }
    
    float z_rel = z_baro - z_baro0;
    z_baro_filt = baroAlpha * z_baro_filt + (1.0f - baroAlpha) * z_rel;

    kalmanPredict(a_z, dt);
    kalmanUpdate(z_baro_filt);
  }

  // Air Data (1 Hz to prevent blocking the IMU loop)
  if (nowMs - lastAirDataUpdate >= AIRDATA_INTERVAL_MS) {
    lastAirDataUpdate = nowMs;
    if (bme680.performReading()) {
      air_temp = bme680.temperature;
      air_hum  = bme680.humidity;
      air_gas  = bme680.gas_resistance;
    }
  }

  server.handleClient();
}
