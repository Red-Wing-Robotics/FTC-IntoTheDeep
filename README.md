# Red Wing Robotics

A first year FTC Robotics team from Chattanooga, TN.

## 2024-2025 Team
- Coach Matt
- Kaden Tucker
- Sam Thompson
- Luke Rainer
- Luke Woolley

## Deploying to the Control Hub

To deploy to the Control Hub, you will first need to connect to the Control Hub via Wifi. The Wifi network should be `FTC-9nB9`.

### Connection on a Mac

To connect to the Control Hub on a Mac, run the following commands:

```bash
cd ~/Library/Android/sdk/platform-tools/
./adb connect 192.168.43.1
./adb devices
```
Once this script runs, you should see the device connected. You should also see the `Rev Control Hub` listed as a target in Android Studio.

### Connection on Windows

To connect to the Control Hub on a Windows machine, run the following commands:

```bash
cd %HOMEPATH%
cd "AppData\Local\Android\sdk\platform-tools\"
adb connect 192.168.43.1
adb devices
```

Once these commands run, you should see the device connected. You should also see the `Rev Control Hub` listed as a target in Android Studio.

### Troubleshooting Connection Issues

If you have connected to the Control Hub via Wifi from a computer, make sure that you turn it off and on again before you try to connect to it from a different computer.