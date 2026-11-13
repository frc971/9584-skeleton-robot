package frc.robot.auto;

import java.util.Collections;
import java.util.List;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

/**
 * Autonomous routines, followed with BLine.
 *
 * <p>Same design as 971's skeleton robot and the training course's BLine lesson, so it reads the
 * same to anyone who has done that lesson:
 * <ul>
 *   <li>A routine is an ordered list of path files from {@code src/main/deploy/autos/paths/}
 *       (made with the BLine GUI). Add one to {@link #AUTO_ROUTINES}.</li>
 *   <li>The first path resets the robot's pose to its own start; later paths continue from
 *       wherever the robot is.</li>
 *   <li>Paths are drawn for the blue alliance and flipped automatically on red. A routine marked
 *       {@code canMirror} also gets a "(Mirrored)" entry for the other side of the field.</li>
 *   <li>The selected routine is loaded while disabled, so pressing enable does not stall on file
 *       reads.</li>
 * </ul>
 *
 * <p>Mechanism actions during a path are BLine event triggers: put an {@code event_trigger} with a
 * {@code lib_key} in the path (BLine GUI), and register what it does with
 * {@link #registerEvent(String, Command)} before autos are built (in RobotContainer).
 *
 * <p>Logged: {@code Auto/Selected}, {@code Auto/SelectedLoaded}, {@code Auto/StartPose}, and
 * everything BLine reports while following, under {@code Auto/FollowPath/...} (path, robot track,
 * cross-track error, rotation error, remaining distance).
 */
public class Autos {

    /**
     * @param canMirror    also offer a mirrored copy (the same routine on the other side of the field)
     * @param displayLabel what the dashboard shows
     * @param pathNames    path files (without .json) in the order they run
     */
    public record AutoRoutine(boolean canMirror, String displayLabel, List<String> pathNames) {
        public AutoRoutine {
            pathNames = List.copyOf(pathNames);
        }
    }

    /** The routines on the dashboard. "None" (do nothing) is always there and is the default. */
    public static final List<AutoRoutine> AUTO_ROUTINES = List.of(
        // Drives 1 m forward. A mobility check, and what ./gradlew simSmoke runs.
        new AutoRoutine(false, "Example Forward 1m", List.of("Example Forward 1m"))
    );

    /** Path-following gains (translation, rotation, cross-track). 971's and 2026 9584's values. */
    private static final double kTranslationP = 5.0;
    private static final double kRotationP = 3.0;
    private static final double kCrossTrackP = 2.0;

    private static final class AutoOption {
        final AutoRoutine routine;
        final boolean mirrored;

        AutoOption(AutoRoutine routine, boolean mirrored) {
            this.routine = routine;
            this.mirrored = mirrored;
        }

        @Override
        public String toString() {
            return routine.displayLabel() + (mirrored ? " (Mirrored)" : "");
        }
    }

    /**
     * Registers what an event trigger key in a path does. Call before the auto runs, for every
     * {@code lib_key} the season's paths use. An unregistered key does nothing.
     */
    public static void registerEvent(String key, Command command) {
        FollowPath.registerEventTrigger(key, command);
    }

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SendableChooser<AutoOption> chooser = new SendableChooser<>();
    private final FollowPath.Builder firstPathBuilder;
    private final FollowPath.Builder nextPathBuilder;

    private AutoOption loadedOption = null;
    private boolean loaded = false;
    private List<Path> loadedPaths = Collections.emptyList();
    private Pose2d loadedStartPose = null;

    public Autos(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        routeBLineLogging();

        chooser.setDefaultOption("None", null);
        for (AutoRoutine routine : AUTO_ROUTINES) {
            AutoOption normal = new AutoOption(routine, false);
            chooser.addOption(normal.toString(), normal);
            if (routine.canMirror()) {
                AutoOption mirrored = new AutoOption(routine, true);
                chooser.addOption(mirrored.toString(), mirrored);
            }
        }
        SmartDashboard.putData("Auto Mode", chooser);

        firstPathBuilder = newPathBuilder(drivetrain).withPoseReset(drivetrain::resetPose);
        nextPathBuilder = newPathBuilder(drivetrain);
    }

    private FollowPath.Builder newPathBuilder(CommandSwerveDrivetrain drivetrain) {
        // CTRE's state.Speeds and ApplyRobotSpeeds are both robot-relative, which is what BLine expects.
        return new FollowPath.Builder(
                drivetrain,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                speeds -> drivetrain.setControl(applyRobotSpeeds.withSpeeds(speeds)),
                new PIDController(kTranslationP, 0.0, 0.0),
                new PIDController(kRotationP, 0.0, 0.0),
                new PIDController(kCrossTrackP, 0.0, 0.0))
            .withShouldMirror(() -> {
                AutoOption selected = chooser.getSelected();
                return selected != null && selected.mirrored;
            })
            .withDefaultShouldFlip();
    }

    /** BLine reports through plain callbacks; send them into the AdvantageKit log. */
    private static void routeBLineLogging() {
        FollowPath.setDoubleLoggingConsumer(p -> Logger.recordOutput("Auto/" + p.getFirst(), p.getSecond()));
        FollowPath.setBooleanLoggingConsumer(p -> Logger.recordOutput("Auto/" + p.getFirst(), p.getSecond()));
        FollowPath.setPoseLoggingConsumer(p -> Logger.recordOutput("Auto/" + p.getFirst(), p.getSecond()));
        FollowPath.setTranslationListLoggingConsumer(p -> Logger.recordOutput("Auto/" + p.getFirst(), p.getSecond()));
    }

    /**
     * Call every loop while disabled. Loads the selected routine's path files if the selection
     * changed, and logs the selection and where the robot should be placed.
     */
    public void whileDisabled() {
        load();
        Logger.recordOutput("Auto/Selected", selectedName());
        Logger.recordOutput("Auto/SelectedLoaded", loaded);
        Pose2d start = startPoseForAlliance();
        if (start != null) {
            Logger.recordOutput("Auto/StartPose", start);
        }
    }

    /**
     * The command for the selected routine; {@code Commands.none()} for "None".
     *
     * <p>The routine ends by actively holding zero speed until auto is over, instead of handing the
     * drivetrain back to the idle drive command. BLine v0.8.4 finishes a path as soon as the robot
     * is within the end tolerance, whether or not it is still moving (its isFinished has a "TODO add
     * final velocity tolerance"), so something has to stop the robot.
     *
     * <p>Measured in simulation and NOT yet on a robot: the example 1 m path ends ~0.35 m past its
     * end point. BLine's speed command ramps down correctly; the simulated drivetrain lags the
     * command by ~0.1 s, so it arrives at ~2.5 m/s. Check overshoot on the real robot (compare
     * Drive/Pose with the path's end in AdvantageScope) before tuning gains: tuning against the
     * simulator's drive lag would fit the simulator, not the robot.
     */
    public Command getAutonomousCommand() {
        load();
        if (loadedPaths.isEmpty()) {
            return Commands.none();
        }
        Command paths = Commands.sequence(
            IntStream.range(0, loadedPaths.size())
                .mapToObj(i -> (i == 0 ? firstPathBuilder : nextPathBuilder).build(loadedPaths.get(i)))
                .toArray(Command[]::new));
        Command stop = drivetrain.applyRequest(() -> applyRobotSpeeds.withSpeeds(new ChassisSpeeds()));
        return paths.andThen(stop).withName("Auto: " + selectedName());
    }

    public String selectedName() {
        AutoOption selected = chooser.getSelected();
        return selected == null ? "None" : selected.toString();
    }

    /** Where to place the robot for the selected routine, for this alliance (null for "None"). */
    public Pose2d startPoseForAlliance() {
        load();
        if (loadedStartPose == null) {
            return null;
        }
        boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        return red ? FlippingUtil.flipFieldPose(loadedStartPose) : loadedStartPose;
    }

    private void load() {
        AutoOption selected = chooser.getSelected();
        if (selected == loadedOption && loaded) {
            return;
        }
        loaded = false;
        loadedOption = selected;
        loadedStartPose = null;
        if (selected == null) {
            loadedPaths = Collections.emptyList();
            loaded = true;
            return;
        }
        try {
            loadedPaths = selected.routine.pathNames().stream().map(Path::new).toList();
        } catch (RuntimeException e) {
            // A missing or broken path file: report it instead of crashing robot code at enable.
            DriverStation.reportError("Auto '" + selected + "' could not load its paths: " + e.getMessage(), false);
            loadedPaths = Collections.emptyList();
            return;
        }
        if (!loadedPaths.isEmpty()) {
            Path first = loadedPaths.get(0).copy();
            if (selected.mirrored) {
                first.mirror();
            }
            loadedStartPose = first.getStartPose();
        }
        loaded = true;
    }
}
