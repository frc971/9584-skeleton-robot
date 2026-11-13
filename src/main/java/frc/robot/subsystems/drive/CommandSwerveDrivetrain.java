package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.drive.sim.MapleSimSwerveDrivetrain;
import frc.robot.subsystems.drive.sim.SimSwerveConstants;
import frc.robot.vision.Vision;

/**
 * The swerve drivetrain: CTRE's generated {@link TunerSwerveDrivetrain} plus the team's overrides,
 * diagnostics, logging, PathPlanner setup and simulation.
 *
 * <p>Regenerating the drivetrain in Tuner X replaces {@code generated/TunerConstants.java} only.
 * Nothing in this file or in {@link SwerveConfigOverrides} needs to change when that happens.
 *
 * <p>Logged signals (all under /RealOutputs/Drive/ unless noted) are listed in docs/logging.md and
 * explained for students in docs/drivetrain-log-guide.md.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Vision.PoseEstimator {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private static final double kBumpTiltThresholdDegrees = 5.0;
    private static final double kOdometryUpdateFrequencyHz = 100.0;
    private static final double kTelemetryUpdateFrequencyHz = 20.0;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SwerveDriveBrake lockWheelsRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;
    /** Per-module currents, temperatures, encoder health, and faults. See docs/drivetrain-log-guide.md. */
    private final SwerveModuleDiagnostics moduleDiagnostics;
    /** Team settings layered over the generated TunerConstants, plus a check that they reached the motors. */
    private final SwerveConfigOverrides configOverrides;

    /**
     * Wheels + gyro only, never corrected by vision. Logged as Drive/OdometryPose next to the fused
     * Drive/Pose, so the log can separate "odometry drifted" from "vision moved the pose".
     */
    private final SwerveDriveOdometry rawOdometry;

    private Notifier m_simNotifier = null;
    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        // regulate...: in simulation only, strips the real robot's encoder offsets and inversions.
        // Without it every simulated wheel points somewhere arbitrary and they fight each other
        // (the 2026 robot never called it). It does nothing on a real robot.
        super(drivetrainConstants, kOdometryUpdateFrequencyHz,
            SwerveConfigOverrides.apply(MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)));

        pitchSignal = getPigeon2().getPitch();
        rollSignal = getPigeon2().getRoll();
        moduleDiagnostics = new SwerveModuleDiagnostics(getModules());
        configOverrides = new SwerveConfigOverrides(getModules());

        BaseStatusSignal.setUpdateFrequencyForAll(kTelemetryUpdateFrequencyHz, pitchSignal, rollSignal);
        BaseStatusSignal.refreshAll(pitchSignal, rollSignal);

        SwerveDriveState state = getState();
        rawOdometry = new SwerveDriveOdometry(getKinematics(), state.RawHeading, state.ModulePositions, state.Pose);

        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    pathApplyRobotSpeeds
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),   // translation
                    new PIDConstants(7, 0, 0)),   // rotation
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /** Returns a command that applies the given control request to the drivetrain every loop. */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command lockWheelsCommand() {
        return run(() -> setControl(lockWheelsRequest));
    }

    public void requestIdle() {
        setControl(idleRequest);
    }

    /** Field-relative "forward" for the current alliance becomes the robot's current heading. */
    public void zeroHeading() {
        Rotation2d forward = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? kRedAlliancePerspectiveRotation
            : kBlueAlliancePerspectiveRotation;
        resetRotation(forward);
        seedFieldCentric();
    }

    // ---- Vision.PoseEstimator ----

    @Override
    public Pose2d pose() {
        return getState().Pose;
    }

    @Override
    public double yawRateRadPerSec() {
        return getState().Speeds.omegaRadiansPerSecond;
    }

    /**
     * Vision timestamps come from the Limelight on the FPGA clock; the Phoenix pose estimator runs
     * on its own clock. This is the ONE place that converts between them.
     * <p>SystemCore has no FPGA. When porting to 2027, re-check this conversion first: a wrong time
     * base applies every vision measurement at the wrong moment and nothing errors. See
     * docs/porting-2027.md.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // ---- periodic + logging ----

    @Override
    public void periodic() {
        applyOperatorPerspective();

        SwerveDriveState state = getState();
        Pose2d odometryPose = rawOdometry.update(state.RawHeading, state.ModulePositions);

        if (mapleSimSwerveDrivetrain != null) {
            // In simulation the physics model is the truth; keep the estimator on it.
            Pose2d simPose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
            super.resetPose(simPose);
            Logger.recordOutput("Drive/Pose", simPose);
        } else {
            Logger.recordOutput("Drive/Pose", state.Pose);
        }
        Logger.recordOutput("Drive/OdometryPose", odometryPose);
        Logger.recordOutput("Drive/VisionCorrectionMeters",
            state.Pose.getTranslation().getDistance(odometryPose.getTranslation()));
        Logger.recordOutput("Drive/GyroYawDegrees", state.RawHeading.getDegrees());
        Logger.recordOutput("Drive/OdometryFrequencyHz", 1.0 / state.OdometryPeriod);

        Logger.recordOutput("Drive/Pose3d", getPose3d());
        Logger.recordOutput("Drive/X", state.Pose.getX());
        Logger.recordOutput("Drive/Y", state.Pose.getY());
        Logger.recordOutput("Drive/Rotation3d", getRobotRotation3d());
        Logger.recordOutput("Drive/PitchDegrees", getPitchDegrees());
        Logger.recordOutput("Drive/RollDegrees", getRollDegrees());
        Logger.recordOutput("Drive/TiltMagnitudeDegrees", getTiltMagnitudeDegrees());
        Logger.recordOutput("Drive/OnBump", isRobotOnBump());

        moduleDiagnostics.logPeriodic();
        if (DriverStation.isDisabled()) {
            // Reads configs back over CAN, so only while disabled to avoid blocking the drive loop.
            configOverrides.verifyPeriodic();
        }

        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
        Logger.recordOutput("Drive/TargetStates", state.ModuleTargets);
        Logger.recordOutput("Drive/MeasuredStates", state.ModuleStates);
        Logger.recordOutput("Drive/MeasuredSpeeds", state.Speeds);
    }

    /**
     * Applies the operator perspective once at startup (so a mid-match code restart drives
     * correctly) and then only while disabled (so driving never changes under the driver).
     */
    private void applyOperatorPerspective() {
        if (m_hasAppliedOperatorPerspective && !DriverStation.isDisabled()) {
            return;
        }
        DriverStation.getAlliance().ifPresent(allianceColor -> {
            setOperatorPerspectiveForward(
                allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation);
            if (!m_hasAppliedOperatorPerspective) {
                seedFieldCentric();
            }
            m_hasAppliedOperatorPerspective = true;
        });
    }

    /** Snapshot of sticky faults, logged once on disable so a fault that cleared is still recorded. */
    public void logStickyFaults() {
        for (var module : getModules()) {
            int driveId = module.getDriveMotor().getDeviceID();
            int steerId = module.getSteerMotor().getDeviceID();
            Logger.recordOutput("Drive/Faults/Drive" + driveId, module.getDriveMotor().getStickyFaultField().getValueAsDouble());
            Logger.recordOutput("Drive/Faults/Steer" + steerId, module.getSteerMotor().getStickyFaultField().getValueAsDouble());
        }
    }

    public double getPitchDegrees() {
        return pitchSignal.getValue().in(Degrees);
    }

    public double getRollDegrees() {
        return rollSignal.getValue().in(Degrees);
    }

    public double getTiltMagnitudeDegrees() {
        return Math.hypot(getPitchDegrees(), getRollDegrees());
    }

    public boolean isRobotOnBump() {
        return getTiltMagnitudeDegrees() >= kBumpTiltThresholdDegrees;
    }

    public Rotation3d getRobotRotation3d() {
        Rotation3d pigeonRotation = getPigeon2().getRotation3d();
        double yawRadians = getState().Pose.getRotation().getRadians();
        return new Rotation3d(pigeonRotation.getX(), pigeonRotation.getY(), yawRadians);
    }

    public Pose3d getPose3d() {
        Pose2d pose2d = getState().Pose;
        return new Pose3d(pose2d.getX(), pose2d.getY(), 0.0, getRobotRotation3d());
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            // Push one sim update instead of blocking the main loop
            mapleSimSwerveDrivetrain.update();
        }
        super.resetPose(pose);
        SwerveDriveState state = getState();
        rawOdometry.resetPosition(state.RawHeading, state.ModulePositions, pose);
    }

    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            SimSwerveConstants.ROBOT_MASS,
            SimSwerveConstants.BUMPER_LENGTH_X,
            SimSwerveConstants.BUMPER_LENGTH_Y,
            SimSwerveConstants.DRIVE_MOTOR_WHEEL,
            SimSwerveConstants.STEER_MOTOR_WHEEL,
            SimSwerveConstants.WHEEL_COF,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
        // Run simulation faster than the robot loop so PID gains behave more realistically
        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);

        // Start the sim robot in open floor on the blue side, facing red. maple-sim models the
        // season's field structures (2026: the tower at the middle of the alliance wall, the hub
        // at x~4.6), so a robot started next to one just pushes against it. Re-check every season.
        Pose2d initialSimPose = new Pose2d(2.0, 2.5, Rotation2d.kZero);
        mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(initialSimPose);
        resetPose(initialSimPose);
    }
}
