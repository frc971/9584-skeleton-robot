package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The heading the robot should face, in field coordinates (0 = toward the red alliance wall,
 * counterclockwise positive), whatever the alliance.
 *
 * <p>The driver's rotation stick moves the target at a rate; when the stick is centered the target
 * stays put and the drivetrain's heading controller holds the robot on it. A command that wants the
 * robot to face a specific direction (snap to an angle, face a scoring station) calls {@link #set}.
 *
 * <p>The stick can only move the target {@code maxLeadRadians} ahead of the robot. Without that, a
 * robot that is blocked, or slower than the stick asks, would build up a target far ahead and whip
 * around once it is free. {@link #set} is not limited, so a snap can ask for any angle.
 */
public final class HeadingTarget {
    private final double maxLeadRadians;
    private Rotation2d target = null;

    public HeadingTarget(double maxLeadRadians) {
        this.maxLeadRadians = maxLeadRadians;
    }

    /**
     * Moves the target by {@code rateRadPerSec × dtSeconds} and returns it. With no target yet, it
     * starts from the robot's current heading.
     */
    public Rotation2d update(Rotation2d robotHeading, double rateRadPerSec, double dtSeconds) {
        if (target == null) {
            target = robotHeading;
        }
        if (rateRadPerSec != 0.0) {
            double lead = target.minus(robotHeading).getRadians() + rateRadPerSec * dtSeconds;
            target = robotHeading.plus(Rotation2d.fromRadians(MathUtil.clamp(lead, -maxLeadRadians, maxLeadRadians)));
        }
        return target;
    }

    public void set(Rotation2d heading) {
        target = heading;
    }

    /** Forget the target. The next {@link #update} starts again from the robot's heading. */
    public void clear() {
        target = null;
    }

    public Optional<Rotation2d> get() {
        return Optional.ofNullable(target);
    }
}
