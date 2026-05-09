# Hardware Status

Last updated: 2026-05-09

## Controller

- Host: Windows laptop
- Python environment: conda `soarm101`
- Python version used during debug: 3.11.15
- Serial dependency: `pyserial==3.5`
- Adapter: Waveshare Bus Servo Adapter (A)
- Adapter mode: USB control mode, jumper at B
- Local serial port during debug: `COM5`
- Baudrate: `1000000`

## Servos

- Servo model: STS3215
- Model number returned by ping: `777`
- External power is required. USB is used only for serial communication.

## ID Map

| ID | Joint | Description | Status |
| --- | --- | --- | --- |
| 1 | shoulder_pan | base rotation | installed, ping OK, read OK, jog OK |
| 2 | shoulder_lift | shoulder lift | installed, ping OK, read OK, jog OK |
| 3 | elbow_flex | elbow flex | installed, ping OK, read OK, jog OK |
| 4 | wrist_flex | wrist pitch | installed, ping OK, read OK, small motion hard to observe |
| 5 | wrist_roll | wrist roll | labeled, not installed yet |
| 6 | gripper | gripper | labeled, not installed yet |

## Current Mechanical Notes

- Only four servos are installed.
- The remaining two servos are blocked by damaged printed parts.
- Large official write demos should not be run before full assembly and calibration.
- ID3 can cause upper/lower arm contact if its amplitude is too large. Use a center offset and small amplitude.
