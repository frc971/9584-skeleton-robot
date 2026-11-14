# Porting to WPILib 2027 and SystemCore

This skeleton builds today on **WPILib 2026** (GradleRIO 2026.2.1, Phoenix 6 26.1.1,
AdvantageKit 26.0.0, BLine-Lib v0.8.4, maple-sim) so it can be built, tested and simulated before the
2027 tools are final. Everything that depends on the roboRIO is in a few known places, listed here.
Do them in this order once the 2027 WPILib, Phoenix 6, AdvantageKit and LimelightLib releases are
out. None of this has been tried yet.

## 1. Tools and vendor libraries

- `build.gradle`: GradleRIO version, and the `roborio` deploy target becomes SystemCore's.
- `.wpilib/wpilib_preferences.json`: `projectYear`.
- `vendordeps/`: replace every file with its 2027 version. maple-sim may lag the others; if it
  does, simulation is what breaks, not the robot. BLine-Lib is built against WPILib 2026.1.1 and
  comes from JitPack (not a WPILib-hosted repo): check it has a 2027 release, and re-run
  `./gradlew simSmoke` to see the example auto still follows its path.
- `.github/workflows/gradle.yml`: the build container image.
- Run `./gradlew build` and `./gradlew test` and fix what breaks.

## 2. Vision timestamps (the one that fails silently)

The generated `CommandSwerveDrivetrain.addVisionMeasurement` converts the Limelight's capture time with
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

- Regenerate the drivetrain in Tuner X and copy `generated/TunerConstants.java`,
  `subsystems/CommandSwerveDrivetrain.java` and `Telemetry.java` over the old ones. Do not edit
  them; team changes go in `Drivetrain` and `SwerveConfigOverrides`. Read the diff: if CTRE changed
  how `addVisionMeasurement` converts timestamps, that is section 2 above.
- Camera positions and names in `Constants.VisionConstants`.
- The field: WPILib's default AprilTag layout updates with WPILib, and the vision filter's field
  size follows it. maple-sim's field structures change too, so re-check the simulation start pose in
  `Drivetrain.startMapleSim` (a robot started inside a structure just pushes on it).
- BLine: robot size and the default speed/acceleration limits in
  `src/main/deploy/autos/config.json`, and the season's paths in `src/main/deploy/autos/paths/`.
- `docs/analyzer-robot.toml`: CAN IDs, limits, camera names, field size. Copy it to the analyzer.
