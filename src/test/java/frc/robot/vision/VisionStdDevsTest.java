package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;

class VisionStdDevsTest {
    private final VisionStdDevs stdDevs = new VisionStdDevs(VisionStdDevs.Config.defaults());

    private static VisionMeasurement at(double dist, int tags) {
        return new VisionMeasurement("ll", new Pose2d(), 0, tags, dist, 0, 0, true);
    }

    @Test
    void matches2026Curve() {
        // 0.02 + 0.02 * 3^2 = 0.2 for two tags
        assertEquals(0.2, stdDevs.xyMeters(at(3.0, 2)), 1e-9);
        assertEquals(0.4, stdDevs.xyMeters(at(3.0, 1)), 1e-9);
        assertEquals(0.14, stdDevs.xyMeters(at(3.0, 5)), 1e-9);
    }

    @Test
    void trustFallsWithDistance() {
        assertTrue(stdDevs.xyMeters(at(4.0, 2)) > stdDevs.xyMeters(at(2.0, 2)));
    }

    @Test
    void megaTag2HeadingIsIgnored() {
        assertEquals(VisionStdDevs.kIgnore, stdDevs.forMeasurement(at(2.0, 2)).get(2, 0));
    }
}
