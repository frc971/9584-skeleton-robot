# What the robot logs, and the contract with the log analyzer

Logs are AdvantageKit `.wpilog` files, one per robot power-on, written to the roboRIO's USB stick
(`/media/sda1/`). During an FMS match AdvantageKit renames the file with the event and match, for
example `akit_26-11-08_18-23-38_mttd_q68.wpilog`. Open them in AdvantageScope, or run the
[9584-log-analyzer](https://github.com/frc971/9584-log-analyzer) on them.

**The analyzer reads signals by name.** Renaming one silently turns a check off: it reports "not
enough data" instead of failing. So:

- Do not rename a signal in the tables below without changing the analyzer in the same week.
- Add new signals freely.
- The analyzer's robot file for this code is `docs/analyzer-robot.toml`. Copy it to the analyzer's
  `robot/` folder and keep it in step with CAN IDs, limits and camera names.

Check the contract any time with a scripted sim match:

```
./gradlew simSmoke
python3 ../9584-log-analyzer/tools/match_report.py logs/<newest>.wpilog --robot docs/analyzer-robot.toml
```

Expected in a sim log: a **PROBLEM** that the swerve config check never passed, and config read
timeouts. Simulated motors do not answer config reads; a real robot does. Everything else should
come back clean.

All `Drive/...` and `Vision/...` names below are under `/RealOutputs/`.

## Read by the analyzer today (names kept from the 2026 robot)

| Signal | Written by | Used for |
|---|---|---|
| `Drive/Pose` | Drivetrain | fused pose, pose jumps |
| `Drive/MeasuredSpeeds`, `Drive/TargetStates`, `Drive/MeasuredStates` | Drivetrain | speed, heading-hold check |
| `Drive/Modules/<Module>/...` (speeds, angles, currents, temps, `Connected`, `Faults`, `Encoder/MagnetHealth`, `Steer/ErrorDegrees`) | SwerveModuleDiagnostics | module comparison, device health |
| `Drive/Config/Verified`, `Drive/Config/Mismatches` | SwerveConfigOverrides | config reached the motors |
| `Drive/Input/Forward`, `Left`, `Rotation`, `State` | TeleopDrive | when the driver was driving; heading hold. `State` values (`IDLING`, `DRIVING_WITH_MANUAL_ROTATION`, `DRIVING_WITH_LOCKED_HEADING`) are matched as strings |
| `BatteryVoltage`, `Power/TotalCurrent`, `Power/Voltage` | Drivetrain, Robot | battery sag, resistance |
| `Vision/Poses`, `TagCounts`, `AvgTagDists`, `StdDevsXY`, `StdDevsTheta`, `Latencies`, `EstimateCount` | Vision | estimate quality, bad solves |
| `Vision/GyroYawDegrees` | Vision | the heading sent to the cameras (also used as "how the robot rotated") |
| `Console` | AdvantageKit | timeline; `System.out.println` lines land here |

AdvantageKit records `/DriverStation/*`, `/SystemStats/*` and `LoggedRobot/FullCycleMS` by itself.

## New in this skeleton (asked for in the analyzer's GEMINI.md)

The analyzer's vision check, its cameras check and `vision_replay.py` read these when they are in
a log (with `--robot robot/2027.toml`), so renaming one has the same cost as the table above.

| Signal | Meaning |
|---|---|
| `Vision/Accepted` (bool[]) | whether each estimate in `Vision/Poses` was applied |
| `Vision/RejectReasons` (string[]) | `ACCEPTED` or why not: `NO_TAGS`, `OFF_FIELD`, `HEADING_MISMATCH`, `SPINNING`, `TOO_FAR`, `AMBIGUOUS`, `LARGE_CORRECTION`, `FIELD_CENTER` (see `vision/RejectReason.java`) |
| `Vision/ResidualMeters`, `Vision/ResidualDegrees` | how far each estimate was from the fused pose when it arrived |
| `Vision/EstimateCameras`, `CaptureTimestamps`, `Ambiguities` | which camera, when, and its worst single-tag ambiguity |
| `Vision/AcceptedPoses` | only the applied estimates (handy in AdvantageScope) |
| `Vision/ReasonCounts/<REASON>` | running total per reason; **authoritative** for counts (see the note below) |
| `Vision/Cameras/<name>/Connected`, `Fps`, `CpuTempC`, `CpuUsage`, `RamPercent`, `Pipeline` | camera health (Connected = its heartbeat is advancing) |
| `Vision/Cameras/<name>/MegaTag1HeadingErrorDeg`, `Vision/GyroDriftDegrees` | while disabled: gyro vs the heading the tags give (MegaTag2 cannot see gyro drift) |
| `Vision/Rewind/SaveCount`, `Vision/Rewind/LastReason` | Limelight Rewind video saves (match end, or the Start button) |
| `Vision/SnapshotCount`, `Vision/LastSnapshot` | still frames saved when an estimate was rejected for HEADING_MISMATCH, LARGE_CORRECTION, OFF_FIELD or FIELD_CENTER |
| `Drive/OdometryPose` | wheels + gyro only, never corrected by vision |
| `Drive/VisionCorrectionMeters` | distance between `Drive/Pose` and `Drive/OdometryPose`: how far vision has moved the pose overall |
| `Drive/GyroYawDegrees` | raw Pigeon heading (unaffected by pose resets) |
| `Drive/Input/TargetHeadingDegrees`, `Drive/Input/HeadingErrorDegrees` | the heading `TeleopDrive` is holding (field frame) and how far the robot is from it; both 0 at rest |
| `Auto/Selected`, `Auto/SelectedLoaded`, `Auto/StartPose` | while disabled: the chosen auto, whether its path files loaded, and where to place the robot (already flipped for red) |
| `Auto/FollowPath/...` | everything BLine reports while following a path: `pathTranslations` (the path), `robotTranslations` (where the robot went), `crossTrackError`, `rotationErrorDeg`, `remainingPathDistanceMeters`, `finished`, event trigger counts, and more |

### Reading the per-estimate arrays

The arrays above line up with `Vision/Poses`: entry *i* of every array describes estimate *i*. They
hold only NEW estimates (a repeat of the last camera frame is counted in
`ReasonCounts/DUPLICATE` and not logged again).

AdvantageKit writes a value only when it changes. Two loops that both log `["ACCEPTED"]` produce
one entry, not two. So **do not count entries in `Vision/RejectReasons` to get totals.** Index by the
times `Vision/Poses` changed and read the other arrays at those times (`series.at(t)` in the
analyzer's `wpilog.py`), or use `Vision/ReasonCounts/*`.
