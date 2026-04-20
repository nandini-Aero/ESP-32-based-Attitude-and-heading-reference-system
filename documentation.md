***

### `DOCUMENTATION.md`

```markdown
# Technical Documentation

This document outlines the software architecture, mathematical models, and API endpoints utilized in the ESP32 AHRS project.

## 1. System Architecture
The firmware operates on a single-threaded loop with precise microsecond timing to maintain a consistent integration step ($dt$). 
* **Sensor Polling:** The main sensor fusion loop is locked to 20 Hz ($50\text{ ms}$ intervals).
* **Barometer Throttling:** Since I2C barometers are relatively slow, the BMP180 polling is divided by 3, resulting in an update rate of approximately $6-7\text{ Hz}$.
* **Web Server:** Handled asynchronously at the end of every main loop iteration to prevent blocking the sensor reads.

## 2. Sensor Fusion Mathematics

### 2.1 Attitude Estimation (Complementary Filter)
Roll and pitch are estimated by combining high-frequency gyroscope data with low-frequency accelerometer data.

First, the accelerometer data is low-pass filtered. Then, the angles derived from the accelerometer are calculated:
$$\phi_{acc} = \arctan\left(\frac{a_y}{a_z}\right)$$
$$\theta_{acc} = \arctan\left(\frac{-a_x}{\sqrt{a_y^2 + a_z^2}}\right)$$

The complementary filter applies an $\alpha$ weight (0.985) to favor the integrated gyro data over the short term, and the accelerometer data over the long term to correct drift:
$$\phi_{t} = \alpha (\phi_{t-1} + \omega_x dt) + (1 - \alpha)\phi_{acc}$$

### 2.2 Vertical Acceleration Extraction
To use the accelerometer for altitude prediction, we must isolate the vertical acceleration relative to the earth, independent of the sensor's current orientation. We rotate the accelerometer vector by the estimated roll ($\phi$) and pitch ($\theta$) and subtract gravity ($G$):

$$a_{z,world} = -\sin(\theta)a_x + \sin(\phi)\cos(\theta)a_y + \cos(\phi)\cos(\theta)a_z$$
$$a_{z,linear} = a_{z,world} - 9.81$$

*Note: A deadband of $0.06\text{ m/s}^2$ is applied to $a_{z,linear}$ to prevent the integration of stationary noise.*

### 2.3 Altitude Estimation (Kalman Filter)
A 1D, 2-state Kalman filter fuses the noisy barometric altitude ($z_{baro}$) with the integrated vertical acceleration ($a_{z,linear}$). 

**State Vector:** Contains altitude and vertical velocity.
$$X = \begin{bmatrix} z \\ v \end{bmatrix}$$

**Prediction Step:** Integrates the physics model based on $a_{z,linear}$ and time step $dt$.
$$z_{est} = z_{est} + v_{est}dt + 0.5 a_{z,linear} dt^2$$
$$v_{est} = v_{est} + a_{z,linear} dt$$
The covariance matrix $P$ is also updated with process noise matrices ($Q_{pos}$, $Q_{vel}$).

**Update Step:** Corrects the state estimate when a new barometric reading is available.
The innovation (residual) is calculated as $y = z_{baro} - z_{est}$. The Kalman gain ($K$) is computed using the measurement noise variance ($R_{alt}$), and the states are updated proportionally.

## 3. Web API Endpoints

The ESP32 runs an HTTP server on port 80.

### `GET /`
Returns the static, minified HTML/CSS/JS dashboard stored in `PROGMEM`.

### `GET /data`
Returns a JSON object containing the latest filtered state variables.

**Response Example:**
```json
{
  "alt": 12.4500,
  "vz": 0.1200,
  "roll": -2.40,
  "pitch": 5.10,
  "yaw": 45.20
}
