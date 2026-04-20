# ESP32 ADAHRS (Air Data, Attitude, and Heading Reference System) ✈️

An advanced firmware stack transforming an ESP32 into a fully functional ADAHRS. This project combines orientation tracking with external environmental data, utilizing high-precision sensors for applications in experimental flight mechanics, aerodynamics, and robotics.

## Sensor Architecture
* **MPU6050:** 6-DOF IMU for attitude (roll, pitch, yaw) and 3D kinematics calculation.
* **BMP388:** Next-generation barometer providing highly accurate, low-noise altitude measurements for the vertical Kalman filter.
* **BME680:** Environmental sensor capturing localized air data (temperature, humidity, and VOC gas resistance) to monitor external atmospheric states.

## Features
* **Custom Sensor Fusion:** 1D Kalman filter for vertical speed/altitude, and Complementary filtering for spatial orientation.
* **Kinematic Velocity Tracking:** Calculates 3D linear acceleration and estimates $V_x$ and $V_y$ via dead reckoning (with leaky integration for drift mitigation).
* **Asynchronous Processing:** Multi-rate sensor polling ensures the BME680's gas heating mechanism does not block the 20Hz IMU control loop.
* **Wireless Dashboard:** Built-in web server displaying real-time telemetry over local WiFi.

## Dependencies
* `Adafruit_MPU6050`
* `Adafruit_BMP3XX`
* `Adafruit_BME680`
* `Adafruit_Sensor`

## Author
Nandini Sanavada

## License
MIT License
