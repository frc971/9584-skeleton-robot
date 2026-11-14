package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

class VectorSlewLimiterTest {
    @Test
    void infiniteLimitPassesThrough() {
        VectorSlewLimiter limiter = new VectorSlewLimiter(Double.POSITIVE_INFINITY);
        Translation2d target = new Translation2d(3.0, -2.0);
        assertEquals(target, limiter.calculate(target, 0.02));
        assertEquals(Translation2d.kZero, limiter.calculate(Translation2d.kZero, 0.02));
    }

    @Test
    void limitsTheWholeVectorKeepingDirection() {
        VectorSlewLimiter limiter = new VectorSlewLimiter(5.0); // 0.1 per 20 ms step
        Translation2d out = limiter.calculate(new Translation2d(3.0, 4.0), 0.02);
        assertEquals(0.1, out.getNorm(), 1e-9);
        assertEquals(new Translation2d(3.0, 4.0).getAngle().getDegrees(), out.getAngle().getDegrees(), 1e-9);
    }

    @Test
    void reachesTargetExactly() {
        VectorSlewLimiter limiter = new VectorSlewLimiter(5.0);
        limiter.reset(new Translation2d(0.15, 0.0));
        limiter.calculate(Translation2d.kZero, 0.02);
        assertEquals(Translation2d.kZero, limiter.calculate(Translation2d.kZero, 0.02));
    }
}
