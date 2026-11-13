package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Decides whether one vision estimate should reach the pose estimator, and records why not.
 *
 * <p>This is a plain class with no hardware in it, so every rule is unit tested
 * (src/test/java/frc/robot/vision/VisionFilterTest.java) and can be replayed against a match log
 * with the 9584-log-analyzer's tools/vision_replay.py, which implements the same rules.
 *
 * <p>Rules are checked in a fixed order and the FIRST one that fails is the reason logged. Every
 * threshold is in {@link Config} with the evidence for its default next to it. Change a threshold
 * there, not by adding an if-statement somewhere else.
 */
public final class VisionFilter {

    /**
     * The filter's thresholds. Defaults come from the 2026 robot and its mttd logs; treat them as
     * starting points to re-check against this season's logs, not as facts.
     *
     * @param fieldLengthMeters        from the season's AprilTag field layout
     * @param fieldWidthMeters         from the season's AprilTag field layout
     * @param fieldMarginMeters        how far outside the walls still counts as on the field
     * @param maxHeadingErrorDegrees   MegaTag2 vs gyro (2026: 15, added after mttd q8)
     * @param maxSingleTagYawRateRadPerSec single-tag estimates are dropped above this spin rate
     *                                 (2026: 3.0 rad/s; multi-tag estimates are kept)
     * @param maxTagDistanceMeters     beyond this, tags are too small to solve accurately
     *                                 (971's cos and 9584's early filter both used 5 m)
     * @param maxSingleTagAmbiguity    2026 VisionSubsystem used 0.2
     * @param maxSingleTagCorrectionMeters a single-tag estimate further than this from the current
     *                                 pose is rejected. Multi-tag estimates are exempt so the robot
     *                                 can recover from a wrong pose. (mttd q8 jumps were single-tag
     *                                 estimates 2-5 m off; the analyzer's replay suggests 1.0 m.)
     */
    public record Config(
        double fieldLengthMeters,
        double fieldWidthMeters,
        double fieldMarginMeters,
        double maxHeadingErrorDegrees,
        double maxSingleTagYawRateRadPerSec,
        double maxTagDistanceMeters,
        double maxSingleTagAmbiguity,
        double maxSingleTagCorrectionMeters) {

        public static Config defaults(double fieldLengthMeters, double fieldWidthMeters) {
            return new Config(fieldLengthMeters, fieldWidthMeters, 0.1, 15.0, 3.0, 5.0, 0.2, 1.0);
        }
    }

    /**
     * The verdict on one estimate, plus how far it disagreed with where the robot thought it was.
     * The residuals are logged whether or not the estimate was accepted, because the rejected ones
     * are exactly the ones worth looking at.
     */
    public record Decision(RejectReason reason, double residualMeters, double residualDegrees) {
        public boolean accepted() {
            return reason == RejectReason.ACCEPTED;
        }
    }

    private final Config config;

    public VisionFilter(Config config) {
        this.config = config;
    }

    public Config config() {
        return config;
    }

    /**
     * @param m                   the estimate
     * @param robotPose           the fused pose right now (what this estimate would correct)
     * @param yawRateRadPerSec    how fast the robot is rotating
     * @param lastTimestampSeconds capture time of the last estimate seen from this camera, or NaN
     */
    public Decision evaluate(
        VisionMeasurement m, Pose2d robotPose, double yawRateRadPerSec, double lastTimestampSeconds) {

        double residualMeters = m.pose().getTranslation().getDistance(robotPose.getTranslation());
        double residualDegrees = m.pose().getRotation().minus(robotPose.getRotation()).getDegrees();
        RejectReason reason = check(m, residualMeters, Math.abs(residualDegrees),
            yawRateRadPerSec, lastTimestampSeconds);
        return new Decision(reason, residualMeters, residualDegrees);
    }

    private RejectReason check(VisionMeasurement m, double residualMeters, double headingErrorDegrees,
        double yawRateRadPerSec, double lastTimestampSeconds) {

        if (m.tagCount() <= 0) {
            return RejectReason.NO_TAGS;
        }
        if (m.timestampSeconds() == lastTimestampSeconds) {
            return RejectReason.DUPLICATE;
        }
        if (!onField(m.pose())) {
            return RejectReason.OFF_FIELD;
        }
        if (m.megaTag2() && headingErrorDegrees > config.maxHeadingErrorDegrees()) {
            return RejectReason.HEADING_MISMATCH;
        }
        boolean singleTag = m.tagCount() == 1;
        if (singleTag && Math.abs(yawRateRadPerSec) > config.maxSingleTagYawRateRadPerSec()) {
            return RejectReason.SPINNING;
        }
        if (m.avgTagDistMeters() > config.maxTagDistanceMeters()) {
            return RejectReason.TOO_FAR;
        }
        if (singleTag && m.maxAmbiguity() > config.maxSingleTagAmbiguity()) {
            return RejectReason.AMBIGUOUS;
        }
        if (singleTag && residualMeters > config.maxSingleTagCorrectionMeters()) {
            return RejectReason.LARGE_CORRECTION;
        }
        return RejectReason.ACCEPTED;
    }

    private boolean onField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double margin = config.fieldMarginMeters();
        return Double.isFinite(x) && Double.isFinite(y)
            && x >= -margin && x <= config.fieldLengthMeters() + margin
            && y >= -margin && y <= config.fieldWidthMeters() + margin;
    }
}
