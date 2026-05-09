# Project Structure

This repository is organized so hardware debug scripts, vendor code, and future LeRobot/ROS work do not get mixed together.

```text
.
├─ config/                 Shared configuration files
├─ docs/                   Hardware notes and workflow docs
├─ tools/stservo/          Our STServo debug and demo scripts
├─ vendor/waveshare_stservo/
│  ├─ scservo_sdk/         Waveshare SDK used by STS/SMS servos
│  ├─ STservo_sdk/         Original vendor SDK variant
│  ├─ sms_sts/             Original ST/SMS example scripts
│  └─ scscl/               Original SC example scripts
├─ requirements.txt
└─ README.md
```

## Rules

- Put our own reusable scripts in `tools/`.
- Keep vendor files under `vendor/`.
- Put hardware state and procedure notes in `docs/`.
- Put shared robot configuration in `config/`.
- Do not commit virtual environments, cache files, videos, or large logs.

## Future Folders

When LeRobot work begins, add a dedicated folder such as:

```text
lerobot/
```

When ROS integration begins, add a dedicated workspace/package area such as:

```text
ros/
```
