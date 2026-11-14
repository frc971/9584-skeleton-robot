package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

class StickShapingTest {
    @Test
    void insideDeadbandIsExactlyZero() {
        assertEquals(0.0, StickShaping.shape(0.07, 0.08, 0.3));
        assertEquals(0.0, StickShaping.shape(-0.08, 0.08, 0.3));
    }

    @Test
    void startsFromZeroAtDeadbandEdgeAndReachesOne() {
        assertTrue(StickShaping.shape(0.081, 0.08, 0.3) < 0.001);
        assertEquals(1.0, StickShaping.shape(1.0, 0.08, 0.3), 1e-9);
        assertEquals(-1.0, StickShaping.shape(-1.0, 0.08, 0.3), 1e-9);
    }

    @Test
    void blendSitsBetweenCubeAndLine() {
        // No deadband: 0.5 → 0.3·0.5 + 0.7·0.125
        assertEquals(0.2375, StickShaping.shape(0.5, 0.0, 0.3), 1e-9);
        assertEquals(0.125, StickShaping.shape(0.5, 0.0, 0.0), 1e-9);
        assertEquals(0.5, StickShaping.shape(0.5, 0.0, 1.0), 1e-9);
    }

    @Test
    void vectorKeepsDirectionAndClampsCorners() {
        Translation2d corner = StickShaping.shape(new Translation2d(1.0, 1.0), 0.08, 0.3);
        assertEquals(1.0, corner.getNorm(), 1e-9);
        assertEquals(45.0, corner.getAngle().getDegrees(), 1e-9);
    }

    @Test
    void vectorDeadbandIsRound() {
        // Each axis is under the deadband, but the stick's length (0.1) is not.
        Translation2d diagonal = StickShaping.shape(new Translation2d(0.0707, 0.0707), 0.08, 0.3);
        assertTrue(diagonal.getNorm() > 0.0);
        assertEquals(Translation2d.kZero, StickShaping.shape(new Translation2d(0.05, 0.05), 0.08, 0.3));
    }
}
