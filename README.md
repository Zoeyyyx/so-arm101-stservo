# SO-ARM101 STServo Workspace

Windows-side STS3215 debug workspace for the SO-ARM101 project. This repository currently covers Waveshare Bus Servo Adapter (A) communication, servo ID records, and a small four-servo motion demo. It is structured to grow into later LeRobot and ROS work without mixing our code with vendor examples.

## Layout

```text
config/                  Shared robot configuration
docs/                    Hardware status and project notes
tools/stservo/           Our STServo scripts
vendor/waveshare_stservo/ Waveshare SDK and original examples
requirements.txt         Minimal Python dependency list
```

More detail: [docs/project-structure.md](docs/project-structure.md)

## Environment

Recommended local environment:

```powershell
conda activate soarm101
python --version
python -m pip install -r requirements.txt
python -c "import serial; print(serial.__version__)"
```

The current Windows debug environment used Python 3.11.15 and `pyserial==3.5`.

## Hardware Snapshot

- Adapter: Waveshare Bus Servo Adapter (A)
- Servo: STS3215
- Debug port on this laptop: `COM5`
- Baudrate: `1000000`
- Servo model observed by ping: `777`
- Power: external servo power required; USB is serial communication only

Current hardware state is tracked in [docs/hardware-status.md](docs/hardware-status.md).

## Servo ID Map

The shared ID map is stored in [config/servo_map.json](config/servo_map.json).

| ID | Joint | Description |
| --- | --- | --- |
| 1 | shoulder_pan | base rotation |
| 2 | shoulder_lift | shoulder lift |
| 3 | elbow_flex | elbow flex |
| 4 | wrist_flex | wrist pitch |
| 5 | wrist_roll | wrist roll |
| 6 | gripper | gripper |

## Four-Servo Smooth Demo

Run from the repository root:

```powershell
python .\tools\stservo\smooth_wave_4.py --port COM5 --ids 1 2 3 --center3 120 --amp1 260 --amp2 220 --amp3 60 --speed 180 --acc 20 --yes
```

If ID3 moves toward collision, reverse the center direction:

```powershell
python .\tools\stservo\smooth_wave_4.py --port COM5 --ids 1 2 3 --center3 -120 --amp1 260 --amp2 220 --amp3 60 --speed 180 --acc 20 --yes
```

Do not run large-motion official write demos before mechanical assembly and calibration are complete.

## Collaboration

Common workflow:

```powershell
git pull
git status
git add .
git commit -m "Describe the change"
git push
```

Before changing shared motion scripts, test with dry run first by omitting `--yes`.
