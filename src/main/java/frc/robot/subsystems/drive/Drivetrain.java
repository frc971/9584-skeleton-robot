package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.sim.MapleSimSwerveDrivetrain;
import frc.robot.subsystems.drive.sim.SimSwerveConstants;
import frc.robot.vision.Vision;

/**
 * The team's drivetrain: Tuner X's generated {@link CommandSwerveDrivetrain} plus 9584's overrides,
 * diagnostics, logging and maple-sim physics.
 *
 * <p>{@code generated/TunerConstants.java} and {@code subsystems/CommandSwerveDrivetrain.java} are
 * Tuner X output and are never edited. Each season, generate a new project in Tuner X, copy those
 * two files (and {@code Telemetry.java}) over the old ones, and read the diff to see what CTRE
 * changed. Everything the team adds lives here, so nothing is lost when they are replaced.
 *
 * <p>Things the generated class already does, so they are not repeated here: alliance operator
 * perspective, SysId routines, and converting vision timestamps from the FPGA clock to Phoenix's
 * clock in {@code addVisionMeasurement} (re-check that conversion first when porting to 2027; see
 * docs/porting-2027.md).
 *
 * <p>Logged signals (all under /RealOutputs/Drive/) are listed in docs/logging.md and explained for
 * students in docs/drivetrain-log-guide.md.
 */
public class Drivetrain extends CommandSwerveDrivetrain implements Vision.PoseEstimator {
    /**
     * maple-sim's step size. It is stepped by the generated class's simulation thread, which runs
     * every 4 ms (CommandSwerveDrivetrain.kSimLoopPeriod); keep the two equal.
     */
    private static final double kSimLoopPeriod = 0.004;
    private static final double kBumpTiltThresholdDegrees = 5.0;
    private static final double kOdometryUpdateFrequencyHz = 100.0;
    private static final double kTelemetryUpdateFrequencyHz = 20.0;

    private final SwerveRequest.SwerveDriveBrake lockWheelsRequest = new SwerveRequest.SwerveDriveBrake();
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

    /**
     * Set once the physics model exists. The generated class starts its simulation thread inside its
     * constructor, before this field is assigned, so {@link #updateSimState} must handle null.
     */
    private volatile MapleSimSwerveDrivetrain mapleSim = null;

    /** Counts pose and heading resets; see {@link #headingResetCount()}. */
    private int headingResetCount = 0;

    public Drivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        // regulate...: in simulation only, strips the real robot's encoder offsets and inversions.
        // Without it every simulated wheel points somewhere arbitrary and they fight each other.
        // It does nothing on a real robot.
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

        if (Utils.isSimulation()) {
            startMapleSim();
        }
    }

    public Command lockWheelsCommand() {
        return run(() -> setControl(lockWheelsRequest));
    }

    public void requestIdle() {
        setControl(idleRequest);
    }

    /**
     * Goes up by one every time the pose or heading is reset (auto start, the driver's re-zero
     * button). A command that holds a heading compares this between loops and drops its target when
     * it changes, because the old target is relative to a heading that no longer exists.
     */
    public int headingResetCount() {
        return headingResetCount;
    }

    // ---- Vision.PoseEstimator (addVisionMeasurement is the generated class's) ----

    @Override
    public Pose2d pose() {
        return getState().Pose;
    }

    @Override
    public double yawRateRadPerSec() {
        return getState().Speeds.omegaRadiansPerSecond;
    }

    // ---- periodic + logging ----

    @Override
    public void periodic() {
        super.periodic(); // operator perspective

        SwerveDriveState state = getState();
        Pose2d odometryPose = rawOdometry.update(state.RawHeading, state.ModulePositions);

        MapleSimSwerveDrivetrain sim = mapleSim;
        if (sim != null) {
            // In simulation the physics model is the truth; keep the estimator on it.
            Pose2d simPose = sim.mapleSimDrive.getSimulatedDriveTrainPose();
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

    // ---- resets: all of them bump headingResetCount ----

    @Override
    public void resetPose(Pose2d pose) {
        MapleSimSwerveDrivetrain sim = mapleSim;
        if (sim != null) {
            sim.mapleSimDrive.setSimulationWorldPose(pose);
            // Push one sim update instead of blocking the main loop
            sim.update();
        }
        super.resetPose(pose);
        SwerveDriveState state = getState();
        rawOdometry.resetPosition(state.RawHeading, state.ModulePositions, pose);
        headingResetCount++;
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        super.resetRotation(rotation);
        headingResetCount++;
    }

    @Override
    public void seedFieldCentric() {
        super.seedFieldCentric();
        headingResetCount++;
    }

    @Override
    public void seedFieldCentric(Rotation2d rotation) {
        super.seedFieldCentric(rotation);
        headingResetCount++;
    }

    // ---- simulation ----

    /**
     * Called by the generated class's simulation thread every 4 ms. CTRE's version runs its own
     * simple model; this runs maple-sim instead, which also models collisions with the field.
     * Overriding it is what lets the generated file stay unedited.
     */
    @Override
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        MapleSimSwerveDrivetrain sim = mapleSim;
        if (sim != null) {
            sim.update();
        }
    }

    private void startMapleSim() {
        MapleSimSwerveDrivetrain sim = new MapleSimSwerveDrivetrain(
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

        // Start the sim robot in open floor on the blue side, facing red. maple-sim models the
        // season's field structures (2026: the tower at the middle of the alliance wall, the hub
        // at x~4.6), so a robot started next to one just pushes against it. Re-check every season.
        Pose2d initialSimPose = new Pose2d(2.0, 2.5, Rotation2d.kZero);
        sim.mapleSimDrive.setSimulationWorldPose(initialSimPose);
        mapleSim = sim;
        resetPose(initialSimPose);
    }
}
