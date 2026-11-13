# Porting to WPILib 2027 and SystemCore

This skeleton builds today on **WPILib 2026** (GradleRIO 2026.2.1, Phoenix 6 26.1.1,
AdvantageKit 26.0.0, PathPlanner, maple-sim) so it can be built, tested and simulated before the
2027 tools are final. Everything that depends on the roboRIO is in a few known places, listed here.
Do them in this order once the 2027 WPILib, Phoenix 6, AdvantageKit and LimelightLib releases are
out. None of this has been tried yet.

## 1. Tools and vendor libraries

- `build.gradle`: GradleRIO version, and the `roborio` deploy target becomes SystemCore's.
- `.wpilib/wpilib_preferences.json`: `projectYear`.
- `vendordeps/`: replace every file with its 2027 version. maple-sim may lag the others; if it
  does, simulation is what breaks, not the robot.
- `.github/workflows/gradle.yml`: the build container image.
- Run `./gradlew build` and `./gradlew test` and fix what breaks.

## 2. Vision timestamps (the one that fails silently)

`CommandSwerveDrivetrain.addVisionMeasurement` converts the Limelight's capture time with
`Utils.fpgaToCurrentTime`. That assumes the roboRIO's FPGA clock. SystemCore has no FPGA. If the
time base is wrong, every vision measurement is applied at the wrong moment, the pose gets worse
while the robot moves, and nothing reports an error.

Check: CTRE's and Limelight's 2027 docs for the right conversion. Then drive at speed past tags
with one camera and compare `Vision/Poses` against `Drive/Pose` in AdvantageScope. They should
agree while moving, not only when stopped.

## 3. Logging location

`Robot.kUsbLogPath` is `/media/sda1/`, where the roboRIO mounts a USB stick. Find SystemCore's log
storage, change the path, and update the analyzer README's "Getting a log" section (it gives the
roboRIO address and path).

## 4. Power distribution

`Robot` reads a REV PDH at CAN ID 1 (`Power/TotalCurrent`, `Power/Voltage`). Check that still
works on SystemCore and that the analyzer's power check still gets data.

## 5. Every season, whatever the hardware

- Regenerate `generated/TunerConstants.java` in Tuner X. Do not edit it; team changes go in
  `SwerveConfigOverrides`.
- Camera positions and names in `Constants.VisionConstants`.
- The field: WPILib's default AprilTag layout updates with WPILib, and the vision filter's field
  size follows it. maple-sim's field structures change too, so re-check the simulation start pose in
  `CommandSwerveDrivetrain.startSimThread` (a robot started inside a structure just pushes on it).
- PathPlanner: robot mass, MOI, and `driveCurrentLimit` (keep it equal to
  `SwerveConfigOverrides.kDriveSlipCurrentAmps`) in `src/main/deploy/pathplanner/settings.json`.
- `docs/analyzer-robot.toml`: CAN IDs, limits, camera names, field size. Copy it to the analyzer.
