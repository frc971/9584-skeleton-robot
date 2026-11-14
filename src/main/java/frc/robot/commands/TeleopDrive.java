package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.HeadingTarget;
import frc.robot.util.StickShaping;
import frc.robot.util.VectorSlewLimiter;

/**
 * Field-centric teleop driving with heading hold. The drivetrain's default command.
 *
 * <p><b>How rotation works.</b> The robot always drives with CTRE's FieldCentricFacingAngle request,
 * which turns the robot toward a target heading. The rotation stick does not turn the robot
 * directly: it turns the target (see {@link HeadingTarget}), and the robot follows. A feedforward
 * of the stick's turn rate keeps that from feeling laggy. When the stick is centered the target
 * stops, so the robot holds its heading while driving. That is heading hold.
 *
 * <p><b>At rest</b> (both sticks centered and the robot no longer being commanded to move) the
 * target is forgotten and the drivetrain gets a zero-speed request: the wheels stay pointed where
 * they were, and nothing resists the robot being turned. The next stick input starts a new target
 * from wherever the robot is facing.
 *
 * <p><b>Frames.</b> Stick input is in the driver's frame (forward = away from the driver station).
 * It is rotated into field coordinates here, once, and everything after that (slew limiting, the
 * target heading, the request) is in field coordinates. A future "face this direction" command
 * just sets a field angle on {@link #headingTarget()}.
 *
 * <p><b>Drive motors run open-loop voltage</b> (stick sets volts, not a measured speed). That is
 * what most teams use for teleop. Closed-loop velocity would keep speed steady as the battery sags,
 * but depends on tuned drive gains; autos (BLine) already use closed loop.
 */
public class TeleopDrive extends Command {
    /** Values of Drive/Input/State. The log analyzer reads these strings: do not rename them. */
    private enum State {
        /** At rest: zero-speed request, no target. */
        IDLING,
        /** Rotation stick is moving the target. */
        DRIVING_WITH_MANUAL_ROTATION,
        /** Rotation stick centered: holding the target while translating. */
        DRIVING_WITH_LOCKED_HEADING
    }

    private static final double kDefaultLoopSeconds = 0.02;

    private final Drivetrain drivetrain;
    private final DoubleSupplier forwardStick;
    private final DoubleSupplier leftStick;
    private final DoubleSupplier rotationStick;

    private final VectorSlewLimiter velocityLimiter =
        new VectorSlewLimiter(Driving.kMaxTranslationAccelMetersPerSecSq);
    private final HeadingTarget headingTarget = new HeadingTarget(Driving.kHeadingMaxLead.in(Radians));

    private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withHeadingPID(Driving.kHeadingKP, 0, Driving.kHeadingKD)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withRotationalDeadband(Driving.kHeadingRotationalDeadband);

    /** Zero speed, so the wheels keep their angles (Idle would let them go limp). */
    private final SwerveRequest.FieldCentric restRequest = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private double lastTimestamp;
    private int lastHeadingResetCount;

    /** Sticks are -1 to 1, forward and left positive, counterclockwise rotation positive. */
    public TeleopDrive(Drivetrain drivetrain, DoubleSupplier forwardStick, DoubleSupplier leftStick,
        DoubleSupplier rotationStick) {
        this.drivetrain = drivetrain;
        this.forwardStick = forwardStick;
        this.leftStick = leftStick;
        this.rotationStick = rotationStick;
        addRequirements(drivetrain);
    }

    /** The heading this command holds. A snap-to-angle feature calls {@code set} on it. */
    public HeadingTarget headingTarget() {
        return headingTarget;
    }

    @Override
    public void initialize() {
        headingTarget.clear();
        velocityLimiter.reset(fieldVelocity());
        lastTimestamp = Timer.getTimestamp();
        lastHeadingResetCount = drivetrain.headingResetCount();
    }

    @Override
    public void execute() {
        double now = Timer.getTimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;
        if (dt <= 0 || dt > 0.25) {
            dt = kDefaultLoopSeconds; // first loop after a pause, or a clock hiccup
        }

        // A pose or heading reset makes the old target meaningless.
        int resets = drivetrain.headingResetCount();
        if (resets != lastHeadingResetCount) {
            lastHeadingResetCount = resets;
            headingTarget.clear();
        }

        Translation2d stick = StickShaping.shape(
            new Translation2d(forwardStick.getAsDouble(), leftStick.getAsDouble()),
            Driving.kTranslationDeadband, Driving.kTranslationLinearBlend);
        double rotation = StickShaping.shape(
            rotationStick.getAsDouble(), Driving.kRotationDeadband, Driving.kRotationLinearBlend);

        Translation2d velocity = velocityLimiter.calculate(toField(stick), dt);
        double turnRate = rotation * Driving.kMaxRotationalRate.in(RadiansPerSecond);

        State state;
        double headingErrorDegrees = 0.0;
        if (velocity.getNorm() == 0.0 && rotation == 0.0) {
            state = State.IDLING;
            headingTarget.clear();
            drivetrain.setControl(restRequest);
        } else {
            state = rotation != 0.0 ? State.DRIVING_WITH_MANUAL_ROTATION : State.DRIVING_WITH_LOCKED_HEADING;
            Rotation2d heading = drivetrain.getState().Pose.getRotation();
            Rotation2d target = headingTarget.update(heading, turnRate, dt);
            drivetrain.setControl(driveRequest
                .withVelocityX(velocity.getX())
                .withVelocityY(velocity.getY())
                .withTargetDirection(target)
                .withTargetRateFeedforward(turnRate));
            headingErrorDegrees = target.minus(heading).getDegrees();
        }

        Logger.recordOutput("Drive/Input/Forward", stick.getX());
        Logger.recordOutput("Drive/Input/Left", stick.getY());
        Logger.recordOutput("Drive/Input/Rotation", rotation);
        Logger.recordOutput("Drive/Input/State", state.toString());
        Logger.recordOutput("Drive/Input/HeadingErrorDegrees", headingErrorDegrees);
        Logger.recordOutput("Drive/Input/TargetHeadingDegrees",
            headingTarget.get().map(Rotation2d::getDegrees).orElse(0.0));
    }

    /** Driver-frame stick (-1 to 1) to field-frame velocity (m/s). */
    private Translation2d toField(Translation2d stick) {
        return stick.times(Driving.kMaxSpeed.in(MetersPerSecond))
            .rotateBy(drivetrain.getOperatorForwardDirection());
    }

    /** The robot's current field velocity, so a limiter starting mid-motion does not jerk to zero. */
    private Translation2d fieldVelocity() {
        var speeds = drivetrain.getState().Speeds; // robot-relative
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(drivetrain.getState().Pose.getRotation());
    }
}
