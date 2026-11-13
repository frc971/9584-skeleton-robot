package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * One robot pose estimate from one camera, in a form that does not depend on which camera brand
 * produced it. Everything downstream (filtering, weighting, logging) works on this, so swapping
 * Limelight for another camera only means writing a new {@link VisionCamera}.
 *
 * @param camera            camera name, e.g. "limelight-front"
 * @param pose              robot pose on the field, blue-alliance origin (WPILib convention)
 * @param timestampSeconds  when the image was captured, on the robot's FPGA clock. The drivetrain
 *                          converts it to the Phoenix time base before handing it to the pose
 *                          estimator (see CommandSwerveDrivetrain.addVisionMeasurement).
 * @param tagCount          AprilTags used in this estimate
 * @param avgTagDistMeters  average camera-to-tag distance
 * @param maxAmbiguity      worst single-tag ambiguity reported (0 = certain, 1 = coin flip); 0 if
 *                          the camera does not report it
 * @param latencyMillis     capture + processing latency
 * @param megaTag2          true if the camera solved position from the gyro heading we sent it
 */
public record VisionMeasurement(
    String camera,
    Pose2d pose,
    double timestampSeconds,
    int tagCount,
    double avgTagDistMeters,
    double maxAmbiguity,
    double latencyMillis,
    boolean megaTag2) {}
