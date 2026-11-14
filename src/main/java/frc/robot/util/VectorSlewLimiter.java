package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Limits how fast a 2D value (here, the robot's commanded field velocity) can change.
 *
 * <p>The limit applies to the change of the whole vector, not x and y separately. Limiting the axes
 * separately would make a diagonal start out curved, because the shorter axis reaches its target
 * first.
 *
 * <p>A limit of {@code Double.POSITIVE_INFINITY} turns it off: the output is always the input.
 */
public final class VectorSlewLimiter {
    private final double maxRatePerSecond;
    private Translation2d last = Translation2d.kZero;

    /** @param maxRatePerSecond largest change per second (m/s² for a velocity); infinity disables */
    public VectorSlewLimiter(double maxRatePerSecond) {
        this.maxRatePerSecond = maxRatePerSecond;
    }

    public Translation2d calculate(Translation2d target, double dtSeconds) {
        Translation2d change = target.minus(last);
        double maxChange = maxRatePerSecond * dtSeconds;
        double length = change.getNorm();
        if (length > maxChange) {
            change = change.times(maxChange / length);
        }
        last = last.plus(change);
        return last;
    }

    public void reset(Translation2d value) {
        last = value;
    }
}
