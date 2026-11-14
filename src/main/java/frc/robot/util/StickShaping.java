package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Turns raw stick values (-1 to 1) into drive commands (-1 to 1).
 *
 * <p>Two steps, in order:
 * <ol>
 *   <li><b>Deadband.</b> Anything smaller than the deadband becomes exactly 0, so a stick that does
 *       not quite center does not creep the robot. Above it, the range is stretched so the output
 *       starts from 0 at the deadband edge instead of jumping.</li>
 *   <li><b>Curve.</b> {@code blend·x + (1 − blend)·x³}. The cube gives fine control near center;
 *       the linear part keeps small stick movements from feeling dead. blend = 0 is a pure cube,
 *       1 is a straight line.</li>
 * </ol>
 */
public final class StickShaping {
    private StickShaping() {}

    /** Shapes one axis. */
    public static double shape(double stick, double deadband, double linearBlend) {
        double x = MathUtil.applyDeadband(MathUtil.clamp(stick, -1.0, 1.0), deadband);
        return linearBlend * x + (1.0 - linearBlend) * x * x * x;
    }

    /**
     * Shapes a 2D stick by its length, keeping its direction. Doing x and y separately would square
     * off the deadband and bend diagonals. A stick pushed into a corner can read longer than 1; that
     * is clamped to 1 so diagonals are not faster than straight lines.
     */
    public static Translation2d shape(Translation2d stick, double deadband, double linearBlend) {
        double length = stick.getNorm();
        if (length == 0.0) {
            return Translation2d.kZero;
        }
        return stick.times(shape(Math.min(length, 1.0), deadband, linearBlend) / length);
    }
}
