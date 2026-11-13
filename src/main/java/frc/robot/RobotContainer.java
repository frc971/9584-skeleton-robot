package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Driving;
import frc.robot.Constants.Field;
import frc.robot.Constants.VisionConstants;
import frc.robot.auto.Autos;
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
 * autos). Add the season's mechanisms under subsystems/, bind them in {@link #configureBindings()},
 * and register the events your BLine paths trigger in {@link #registerAutoEvents()}. Buttons already
 * used: back (re-zero field-centric), right stick click (lock wheels), start (save Limelight Rewind
 * video).
 */
public class RobotContainer {
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision vision;
    private final Autos autos;
    private final SwerveTelemetry swerveTelemetry =
        new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond), drivetrain::getRobotRotation3d);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final ManualDriveCommand manualDrive;

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
        registerAutoEvents();
        autos = new Autos(drivetrain);
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

    /**
     * What each BLine event trigger key does. Every {@code lib_key} used in a path file needs an
     * entry here, e.g. {@code Autos.registerEvent("intakeDown", intake.deployCommand());}
     */
    private void registerAutoEvents() {
    }

    public Command getAutonomousCommand() {
        return autos.getAutonomousCommand();
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
        autos.whileDisabled();
    }
}
