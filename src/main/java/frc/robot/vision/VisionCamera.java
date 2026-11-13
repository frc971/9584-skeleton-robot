package frc.robot.vision;

import java.util.Optional;

/**
 * One pose-estimating camera. {@link LimelightCamera} is the only implementation today; the
 * interface exists so a different camera (for example 971's cos on an Orin) could be added
 * without touching filtering, weighting, logging or the drivetrain.
 */
public interface VisionCamera {
    /** NetworkTables / log name, e.g. "limelight-front". */
    String name();

    /**
     * Tell the camera which way the robot faces. Must be called every loop BEFORE reading an
     * estimate; MegaTag2 solves position from this heading. {@link Vision} is the only caller, so
     * there is exactly one place that sets it (the 2026 robot had two, and one sent 0).
     */
    void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSec);

    /** Latest gyro-assisted (MegaTag2) estimate, or empty if the camera sees no tags. */
    Optional<VisionMeasurement> megaTag2();

    /**
     * Latest estimate that solves heading from the tags alone (MegaTag1). Used only to check the
     * gyro for drift, never fed to the pose estimator directly.
     */
    Optional<VisionMeasurement> megaTag1();

    /** Connection and hardware health, for the log. */
    CameraHealth health();

    /** Save the last {@code seconds} of video on the camera (Limelight 4 Rewind). */
    void saveRewind(double seconds);

    /** Save the current frame as a still on the camera. */
    void saveSnapshot();
}
