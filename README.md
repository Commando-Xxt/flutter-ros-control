# Flutter App + ROS Control

This project implements a Flutter mobile app that communicates with a ROS2 robot via **rosbridge** over WebSocket (`ws://`).

---

## Features
- Publishes to `/cmd_vel` to control robot movement.
- Subscribes to `/joint_states` to read robot joint positions.
- Configurable ROS host/port directly from the app UI.

---

## Build & Run (Flutter)
```bash
cd src/flutter_app
flutter pub get
flutter run
```

---

## ROS Setup
See [`ros/run_rosbridge.md`](ros/run_rosbridge.md) for steps to install and run **rosbridge** on ROS2 Humble.

Basic:
```bash
ros2 launch rosbridge_server rosbridge_websocket.launch.xml
```

---

## Expected Behavior
1. **Connect** → Status changes to `Connected`.
2. **Send `/cmd_vel`** → Robot starts moving.
3. **Stop** → Sends zero velocities to stop robot.
4. **Joint States** → Joint names/positions appear if `/joint_states` is published.

---

## Notes
- Allow cleartext in `AndroidManifest.xml` **(dev only)**:
  ```xml
  <uses-permission android:name="android.permission.INTERNET" />
  <application android:usesCleartextTraffic="true" ...>
  ```
- Phone and ROS machine must be on the **same LAN**.
- For Android 12+, add `android:exported="true"` to `<activity>`.

---
