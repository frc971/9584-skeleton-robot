# Vision: Limelight 4 AprilTag localization

The code is in `src/main/java/frc/robot/vision/`:

| File | Job |
|---|---|
| `LimelightCamera` | one Limelight 4: mounting position, heartbeat, health, Rewind, snapshots |
| `VisionCamera` | the interface it implements (another camera type could be added behind it) |
| `VisionFilter` | the rules that reject bad estimates, each with a named reason |
| `VisionStdDevs` | how much to trust each accepted estimate |
| `Vision` | runs every loop: sends heading, reads estimates, filters, feeds the drivetrain, logs |

## Why it works this way

- **MegaTag2 needs the robot's heading every loop.** `Vision` is the only code that sends it. The
  2026 robot had two places sending it, and one sent 0 (fixed by "don't reset gyro to 0", 2026-04-07).
- **Several cameras.** Each camera's estimates go to the pose estimator separately, and the
  estimator combines them. More cameras means tags are in view more often, which matters more than
  frame rate.
- **Every rejected estimate is logged with its reason.** The 2026 log could only show what the
  camera returned, not what the robot did with it.
- **Heading is never corrected by vision.** MegaTag2 heading comes from the gyro, so it is not new
  information. Gyro drift is checked separately while disabled with MegaTag1 (see below).

## Setting up a camera

1. Set the Limelight's hostname to match `Constants.VisionConstants` (`limelight-front`, ...).
   Pipeline 0 = AprilTag, full 3D, with the season's field map.
2. Measure the camera's position from CAD and put it in `Constants.VisionConstants.kCameras`.
   x forward, y left, z up from the center of the robot on the floor. The code sends it to the
   camera at startup, so do not set it in the web UI (the code would overwrite it anyway).
3. Delete entries for cameras the robot does not have. A listed camera that is not connected raises
   a red dashboard alert.
4. Deploy, open each camera's web UI, and check the robot drawn in its 3D view sits where the camera
   thinks it is. Pitch and yaw signs are the usual mistake.

## The known-pose test (do this before trusting any of it)

Put the robot at a few spots on a real field whose positions you know (tape on the carpet, measured
from a wall), at different distances and angles to the tags. At each spot, read `Vision/Poses` and
`Drive/Pose` in AdvantageScope.

- Off by a constant amount everywhere: the camera position in `Constants` is wrong.
- Off more the further away the tags are: that is normal. Note how much, per distance and tag count.
- One camera disagrees with the others: that camera's position or its calibration.

The numbers you write down are what `VisionStdDevs.Config` should be fitted to. The defaults are
the 2026 robot's, which were never fitted to anything, and single-tag estimates were trusted too
much (5-10 cm).

## Tuning the filter

Every threshold is in `VisionFilter.Config.defaults()` with the reason for its value. To test a
change before deploying, run the analyzer's replay tool on a real match log:

```
python3 ../9584-log-analyzer/tools/vision_replay.py logs/<match>.wpilog --max-heading-error 15 --max-correction 1.0
```

It shows which pose jumps a rule would have prevented and how much good data it would have
thrown away. Add a test in `src/test/java/frc/robot/vision/VisionFilterTest.java` for any new rule,
using a real case from a log where you have one (see `mttdFieldCenterBadSolveIsRejected`).

## Gyro drift check

MegaTag2 trusts the gyro completely, so a drifted gyro gives confident, wrong positions that no
filter rule can catch. While disabled, `Vision` compares the gyro to the heading MegaTag1 solves
from the tags (needs 2+ tags in view). More than 5 degrees off raises a dashboard warning to
re-zero heading. It never changes the heading by itself.

## Rewind video and snapshots

Limelight 4 keeps a rolling video buffer. The code saves it:

- automatically when an FMS match ends (165 s, the most the camera accepts, covers the whole match);
- when the driver presses **Start** (last 30 s), for "that looked wrong" moments in practice.

It also saves one still frame when an estimate is rejected for HEADING_MISMATCH,
LARGE_CORRECTION or OFF_FIELD while enabled, at most once every 2 s.

Each save is logged (`Vision/Rewind/*`, `Vision/SnapshotCount`) and printed to the console, so the
log says which match a video belongs to. Watch them in the camera's web UI (Rewind tab), or download
them in the pit. Check the file format on a real camera before writing any download script:
Limelight's docs describe both `.rwnd` and `.avi`.

## Simulation

There is no Limelight in simulation. `./gradlew simSmoke` publishes fake Limelight data on the
real NetworkTables keys (with the 2026 field-center bad solve mixed in), so the whole path runs.
See docs/logging.md.
