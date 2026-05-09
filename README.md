# SO-ARM101 STServo Debug

Windows-side STS3215 / Waveshare Bus Servo Adapter (A) debug workspace for SO-ARM101.

## Environment

Recommended conda environment:

```powershell
conda activate soarm101
python --version
python -c "import serial; print(serial.__version__)"
```

Required package:

```powershell
python -m pip install -r requirements.txt
```

## Hardware

- Adapter: Waveshare Bus Servo Adapter (A)
- Servo: STS3215
- Port used during local debug: `COM5`
- Baudrate: `1000000`
- Servo model observed by ping: `777`

USB only provides serial communication. Servos require external power.

## Current ID Map

| ID | Joint | Description |
| --- | --- | --- |
| 1 | shoulder_pan | base rotation |
| 2 | shoulder_lift | shoulder lift |
| 3 | elbow_flex | elbow flex |
| 4 | wrist_flex | wrist pitch |
| 5 | wrist_roll | wrist roll |
| 6 | gripper | gripper |

## Four-Servo Smooth Demo

Run from `stservo-env\sms_sts`:

```powershell
cd stservo-env\sms_sts
python .\smooth_wave_4.py --ids 1 2 3 --center3 120 --amp1 260 --amp2 220 --amp3 60 --speed 180 --acc 20 --yes
```

If ID3 moves toward collision, reverse the center direction:

```powershell
python .\smooth_wave_4.py --ids 1 2 3 --center3 -120 --amp1 260 --amp2 220 --amp3 60 --speed 180 --acc 20 --yes
```

Do not run large-motion official write demos before mechanical assembly and calibration are complete.
