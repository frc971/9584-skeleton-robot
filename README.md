# 9584 skeleton robot

The starting point for each new season's robot code: a CTRE swerve drivetrain from Tuner X,
Limelight 4 localization, driving, autos, and logging the
[9584-log-analyzer](https://github.com/frc971/9584-log-analyzer) can read. There are no game
mechanisms. Start the season by copying this repo and adding them.

Most of it comes from the 2026 robot (`9584-wcp-code-2026`, branch `competition`), with the
game-specific code removed and the vision rewritten.

## What's here

| Folder | What |
|---|---|
| `generated/TunerConstants.java` | made by Tuner X. Replace it, never edit it |
| `subsystems/drive/` | drivetrain, team overrides on top of Tuner X, per-module diagnostics, simulation |
| `commands/ManualDriveCommand.java` | field-centric driving; holds heading 0.25 s after the driver stops turning |
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
2. Generate the drivetrain in Tuner X and replace `generated/TunerConstants.java`.
3. Measure camera positions from CAD into `Constants.VisionConstants`, then do the known-pose test
   in [docs/vision.md](docs/vision.md).
4. Update `docs/analyzer-robot.toml`, and the robot size and speed limits BLine uses in
   `src/main/deploy/autos/config.json`.
5. Add mechanisms under `subsystems/` and bind them in `RobotContainer`.
6. Draw the season's paths in the BLine GUI, list them in `Autos.AUTO_ROUTINES`, and register
   what each path's event triggers do in `RobotContainer.registerAutoEvents()`. Delete the
   "Example Forward 1m" routine once real ones exist.

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

Apache-2.0 ([LICENSE](LICENSE)). Parts of this code descend from WCP's starter code via the 2026
robot (`9584-wcp-code-2026`), which is MIT-licensed; that notice is kept in
[LICENSE-WCP](LICENSE-WCP) and must stay with the code. WPILib's license is in
[WPILib-License.md](WPILib-License.md).
