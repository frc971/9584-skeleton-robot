# 9584 skeleton robot

The starting point for each new season's robot code: a CTRE swerve drivetrain from Tuner X,
Limelight 4 localization, driving, autos, and logging the
[9584-log-analyzer](https://github.com/frc971/9584-log-analyzer) can read. There are no game
mechanisms. Start the season by copying this repo and adding them.

It started from the 2026 robot (`9584-wcp-code-2026`), with the game-specific code removed, the
vision rewritten, and the drivetrain rebuilt around the files Tuner X generates.

## What's here

| Folder | What |
|---|---|
| `generated/TunerConstants.java`, `subsystems/CommandSwerveDrivetrain.java`, `Telemetry.java` | made by Tuner X. Replace them, never edit them (see below) |
| `subsystems/drive/` | `Drivetrain` (the team's subclass of the generated drivetrain), team overrides on top of Tuner X, per-module diagnostics, maple-sim simulation |
| `commands/TeleopDrive.java` | field-centric driving with heading hold; the rotation stick turns a target heading the robot follows |
| `util/` | stick shaping, the velocity slew limiter, and the target heading `TeleopDrive` uses |
| `auto/Autos.java` | autonomous routines, followed with [BLine](https://github.com/edanliahovetsky/BLine-Lib). Paths are in `src/main/deploy/autos/paths/` |
| `vision/` | Limelight localization. See [docs/vision.md](docs/vision.md) |
| `sim/SimSmoke.java` | scripted headless sim match for checking logs |
| `RobotContainer.java` | where subsystems are built and buttons are bound. Season mechanisms go here |

## Controls (driver, port 0)

| Input | Does |
|---|---|
| Left stick | drive (field-centric) |
| Right stick X | turn |
| Back | re-zero field-centric "forward" |
| Right stick click (hold) | lock wheels in an X |
| Start | save the last 30 s of Limelight Rewind video |

## Build, test, simulate

```
./gradlew build          # compile and run unit tests
./gradlew simulateJava   # simulation with the WPILib GUI (drive with a controller or keyboard)
./gradlew simSmoke       # 20 s scripted match, headless, writes logs/akit_*.wpilog
```

## Logs

Read [docs/logging.md](docs/logging.md) before renaming any logged signal: the analyzer reads them
by name. [docs/drivetrain-log-guide.md](docs/drivetrain-log-guide.md) explains the drivetrain
signals for students.

## Starting a new season

1. Copy this repo to `9584-<game>-<year>`.
2. Generate the drivetrain in Tuner X and copy its three files over the old ones (see
   "Tuner X files" below).
3. Measure camera positions from CAD into `Constants.VisionConstants`, then do the known-pose test
   in [docs/vision.md](docs/vision.md).
4. Update `docs/analyzer-robot.toml`, and the robot size and speed limits BLine uses in
   `src/main/deploy/autos/config.json`.
5. Add mechanisms under `subsystems/` and bind them in `RobotContainer`.
6. Draw the season's paths in the BLine GUI, list them in `Autos.AUTO_ROUTINES`, and register
   what each path's event triggers do in `RobotContainer.registerAutoEvents()`. Delete the
   "Example Forward 1m" routine once real ones exist.

## Tuner X files

Tuner X's swerve generator writes a whole project. Three of its files are used here exactly as
generated: `generated/TunerConstants.java`, `subsystems/CommandSwerveDrivetrain.java` and
`Telemetry.java`. Never edit them. Each season (and after a Phoenix update), generate a fresh
project in Tuner X, copy those three files over, and run `git diff`: that shows exactly what CTRE
changed this year. Do not copy its `RobotContainer`, `Robot` or `Main`; ours replace them.

Everything the team adds is in `subsystems/drive/Drivetrain.java`, which extends the generated
`CommandSwerveDrivetrain`, and in `SwerveConfigOverrides` (current limits and steer feedback the
team changed from Tuner X's values). `RobotContainer` builds `Drivetrain` directly instead of
calling `TunerConstants.createDrivetrain()`.

The simulation is maple-sim, not CTRE's simple model, because it models collisions with the field.
The generated class starts a 4 ms simulation thread that calls `updateSimState`; `Drivetrain`
overrides that method to step maple-sim instead, so the generated file needs no changes for it.

The current `subsystems/CommandSwerveDrivetrain.java` was taken from CTRE's 2026 example project
(`Phoenix6-Examples/java/SwerveWithPathPlanner`) with its PathPlanner setup removed, which is what
the generator writes. The first time someone runs the Tuner X generator, replace it with the real
output and check the diff is empty or small.

## Driving

- **Sticks** go through a deadband and then a curve that blends a cube with a straight line
  (`util/StickShaping.java`). The translation stick is shaped by its length, so the deadband is
  round and diagonals are not faster. Deadbands and blends are in `Constants.Driving`; the
  deadbands are placeholders until measured from a real controller's drift.
- **Heading hold:** the robot always uses CTRE's `FieldCentricFacingAngle`. The rotation stick moves
  a target heading, the robot follows it, and when the stick is centered it stays on it. The stick
  cannot push the target more than 30° ahead of the robot, so a blocked robot does not whip around
  when freed. The heading gains in `Constants.Driving` are placeholders that need tuning on the
  robot. A snap-to-angle command sets a field angle on `TeleopDrive.headingTarget()`.
- **At rest** (both sticks centered) there is no heading hold: the drivetrain gets a zero-speed
  request, so the wheels keep their angles and nothing fights a push. The next input starts a new
  target from wherever the robot faces.
- **Acceleration limit:** `kMaxTranslationAccelMetersPerSecSq` limits how fast the commanded
  velocity changes. It is off (infinity) until it is tuned on the real robot.

## Autos (BLine)

Same structure as 971's skeleton robot and the training course's BLine lesson
(`training-2026`, lesson 6): a routine is a list of path files run in order, the first path resets
the pose to its own start, paths are drawn for blue and flipped on red, and a routine can offer a
mirrored copy. Mechanism actions during a path are BLine event triggers (`lib_key` in the path
file, registered with `Autos.registerEvent`). Everything BLine reports while following is logged
under `Auto/FollowPath/...`; see [docs/logging.md](docs/logging.md).

`./gradlew simSmoke` runs the example auto. In simulation it ends about 0.35 m past the path's end
because the simulated drivetrain lags its speed commands; check overshoot on the real robot before
tuning the path gains in `Autos`.

For WPILib 2027 and SystemCore, see [docs/porting-2027.md](docs/porting-2027.md). This repo builds on
WPILib 2026 until then.

## License

Apache-2.0 ([LICENSE](LICENSE)). WPILib's license is in [WPILib-License.md](WPILib-License.md).
The Tuner X files are CTRE's generated code, and `vision/LimelightHelpers.java` is Limelight's
library, each under its own terms.
