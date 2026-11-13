package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Every rule in VisionFilter, plus the real 2026 failure it exists for. */
class VisionFilterTest {
    private static final double kLength = 16.54;
    private static final double kWidth = 8.07;
    private final VisionFilter filter = new VisionFilter(VisionFilter.Config.defaults(kLength, kWidth));

    private static final Pose2d kRobot = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(30));

    private static VisionMeasurement estimate(double x, double y, double headingDeg, int tags,
        double dist, double ambiguity) {
        return new VisionMeasurement("limelight-front", new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)),
            10.0, tags, dist, ambiguity, 30.0, true);
    }

    private RejectReason reason(VisionMeasurement m, double yawRate, double lastTimestamp) {
        return filter.evaluate(m, kRobot, yawRate, lastTimestamp).reason();
    }

    @Test
    void goodMultiTagEstimateIsAccepted() {
        var d = filter.evaluate(estimate(3.1, 4.0, 30, 2, 2.0, 0.0), kRobot, 0.0, Double.NaN);
        assertTrue(d.accepted());
        assertEquals(0.1, d.residualMeters(), 1e-9);
    }

    @Test
    void noTags() {
        assertEquals(RejectReason.NO_TAGS, reason(estimate(3, 4, 30, 0, 2, 0), 0, Double.NaN));
    }

    @Test
    void sameCaptureTwiceIsDuplicate() {
        assertEquals(RejectReason.DUPLICATE, reason(estimate(3, 4, 30, 2, 2, 0), 0, 10.0));
    }

    @Test
    void offField() {
        assertEquals(RejectReason.OFF_FIELD, reason(estimate(-1.0, 4, 30, 2, 2, 0), 0, Double.NaN));
        assertEquals(RejectReason.OFF_FIELD, reason(estimate(3, kWidth + 0.5, 30, 2, 2, 0), 0, Double.NaN));
        assertEquals(RejectReason.OFF_FIELD, reason(estimate(Double.NaN, 4, 30, 2, 2, 0), 0, Double.NaN));
    }

    /** 2026 mttd q8: field center facing 0 degrees while the gyro said otherwise. 5 m jump. */
    @Test
    void mttdFieldCenterBadSolveIsRejected() {
        var bad = estimate(8.27, 4.03, 0, 2, 3.0, 0.0);
        assertEquals(RejectReason.HEADING_MISMATCH, reason(bad, 0, Double.NaN));
    }

    @Test
    void headingCheckIgnoresWrapAround() {
        Pose2d robot = new Pose2d(3, 4, Rotation2d.fromDegrees(179));
        var m = estimate(3, 4, -179, 2, 2, 0);
        assertTrue(filter.evaluate(m, robot, 0, Double.NaN).accepted());
    }

    @Test
    void headingCheckOnlyAppliesToMegaTag2() {
        var mt1 = new VisionMeasurement("ll", new Pose2d(3, 4, Rotation2d.fromDegrees(90)), 10, 2, 2, 0, 30, false);
        assertTrue(filter.evaluate(mt1, kRobot, 0, Double.NaN).accepted());
    }

    @Test
    void singleTagWhileSpinningIsRejectedButMultiTagIsNot() {
        assertEquals(RejectReason.SPINNING, reason(estimate(3, 4, 30, 1, 2, 0), 4.0, Double.NaN));
        assertEquals(RejectReason.ACCEPTED, reason(estimate(3, 4, 30, 2, 2, 0), 4.0, Double.NaN));
        assertEquals(RejectReason.SPINNING, reason(estimate(3, 4, 30, 1, 2, 0), -4.0, Double.NaN));
    }

    @Test
    void tooFar() {
        assertEquals(RejectReason.TOO_FAR, reason(estimate(3, 4, 30, 2, 6.0, 0), 0, Double.NaN));
    }

    @Test
    void ambiguousSingleTag() {
        assertEquals(RejectReason.AMBIGUOUS, reason(estimate(3, 4, 30, 1, 2, 0.5), 0, Double.NaN));
        assertEquals(RejectReason.ACCEPTED, reason(estimate(3, 4, 30, 2, 2, 0.5), 0, Double.NaN));
    }

    /** mttd q8: single-tag estimates 2-5 m from the pose, correct heading. */
    @Test
    void singleTagLargeCorrectionIsRejectedButMultiTagCanRecover() {
        assertEquals(RejectReason.LARGE_CORRECTION, reason(estimate(6.0, 4, 30, 1, 2, 0), 0, Double.NaN));
        assertEquals(RejectReason.ACCEPTED, reason(estimate(6.0, 4, 30, 2, 2, 0), 0, Double.NaN));
    }

    @Test
    void residualDegreesIsSigned() {
        var d = filter.evaluate(estimate(3, 4, 40, 2, 2, 0), kRobot, 0, Double.NaN);
        assertEquals(10.0, d.residualDegrees(), 1e-6);
    }
}
