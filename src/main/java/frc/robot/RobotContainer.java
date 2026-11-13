package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Driving;
import frc.robot.Constants.Field;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.SwerveTelemetry;
import frc.robot.vision.LimelightCamera;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionCamera;
import frc.robot.vision.VisionFilter;
import frc.robot.vision.VisionStdDevs;

/**
 * Builds the robot: subsystems, controller bindings, autos.
 *
 * <p>This skeleton has the parts that carry over every season (drivetrain, vision, driving,
 * autos). Add the season's mechanisms under subsystems/ and bind them in
 * {@link #configureBindings()}. Buttons already used: back (re-zero field-centric), right stick
 * click (lock wheels), start (save Limelight Rewind video).
 */
public class RobotContainer {
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision vision;
    private final SwerveTelemetry swerveTelemetry =
        new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond), drivetrain::getRobotRotation3d);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final ManualDriveCommand manualDrive;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        vision = new Vision(
            buildCameras(),
            new VisionFilter(VisionFilter.Config.defaults(
                Field.kTagLayout.getFieldLength(), Field.kTagLayout.getFieldWidth())),
            new VisionStdDevs(VisionStdDevs.Config.defaults()),
            drivetrain);

        manualDrive = new ManualDriveCommand(
            drivetrain,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX());
        drivetrain.setDefaultCommand(manualDrive);
        drivetrain.registerTelemetry(swerveTelemetry::telemeterize);

        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private static List<VisionCamera> buildCameras() {
        return VisionConstants.kCameras.stream()
            .map(c -> (VisionCamera) new LimelightCamera(c.name(), c.robotToCamera(), c.pipeline()))
            .toList();
    }

    private void configureBindings() {
        driver.back().onTrue(Commands.runOnce(manualDrive::seedFieldCentric).ignoringDisable(true));
        driver.rightStick().whileTrue(drivetrain.lockWheelsCommand());
        driver.start().onTrue(vision.saveRewindCommand());

        // Season mechanisms: bind them here.
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        if (selected == null) {
            DriverStation.reportWarning("No autonomous command selected", false);
            return Commands.none();
        }
        return selected;
    }

    /**
     * Seeds the pose from the selected PathPlanner auto's starting pose, flipped for red. Without
     * this the robot starts auto wherever the pose happened to be, and the analyzer's vision check
     * reports "Pose just before auto" at the field origin.
     */
    public void resetPoseForAuto() {
        if (!(autoChooser.getSelected() instanceof PathPlannerAuto auto)) {
            return;
        }
        Pose2d startingPose = auto.getStartingPose();
        if (startingPose == null) {
            return;
        }
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            startingPose = FlippingUtil.flipFieldPose(startingPose);
        }
        drivetrain.resetPose(startingPose);
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public void onDisable() {
        drivetrain.requestIdle();
        drivetrain.logStickyFaults();
    }

    public void whileDisabled() {
        drivetrain.requestIdle();
    }
}
