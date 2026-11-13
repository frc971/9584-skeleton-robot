package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.LimelightHelpers.RawFiducial;

/**
 * A Limelight 4 used for AprilTag localization.
 *
 * <p>Everything about how the camera is mounted is set FROM CODE at startup
 * ({@code setCameraPose_RobotSpace}), so the mounting lives in git instead of in each camera's web
 * UI, and swapping a camera body needs no web-UI setup beyond its name and pipeline.
 *
 * <p>NetworkTables keys used directly (not wrapped by LimelightHelpers v1.12), per Limelight's
 * NetworkTables API docs:
 * <ul>
 *   <li>{@code hb}: heartbeat, increases once per frame. Used to detect a camera that stopped.</li>
 *   <li>{@code hw}: [cpu temp C, cpu usage, ram usage %, fps].</li>
 *   <li>{@code rewind_enable_set}: 1 = keep recording the Rewind buffer.</li>
 *   <li>{@code capture_rewind}: [counter, seconds]. Incrementing the counter saves the last
 *       {@code seconds} of video (max 165 via NT, rate-limited to once per 2 s).</li>
 *   <li>{@code snapshot}: incrementing saves one still frame (rate-limited to once per 10
 *       frames).</li>
 * </ul>
 * If a newer LimelightHelpers is vendored, these can switch to its wrappers
 * ({@code triggerRewindCapture}, {@code setRewindEnabled}, {@code triggerSnapshot},
 * {@code getHeartbeat}) with no change in behavior.
 */
public class LimelightCamera implements VisionCamera {
    /** A camera whose heartbeat has not moved for this long is treated as disconnected. */
    private static final double kHeartbeatTimeoutSec = 0.5;
    /** Rewind capture length the camera accepts over NetworkTables. */
    public static final double kMaxRewindSeconds = 165.0;

    private final String name;
    private double lastHeartbeat = Double.NaN;
    private double lastHeartbeatChangeSec = Double.NEGATIVE_INFINITY;
    private double rewindCounter = 0;
    private double snapshotCounter = 0;

    /**
     * @param name           the camera's hostname/NT name, e.g. "limelight-front"
     * @param robotToCamera  camera position relative to robot center, from CAD:
     *                       x forward, y left, z up (meters); roll/pitch/yaw (radians)
     * @param pipeline       AprilTag localization pipeline index
     */
    public LimelightCamera(String name, Transform3d robotToCamera, int pipeline) {
        this.name = name;
        LimelightHelpers.setPipelineIndex(name, pipeline);
        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            robotToCamera.getX(),
            robotToCamera.getY(),
            robotToCamera.getZ(),
            Math.toDegrees(robotToCamera.getRotation().getX()),
            Math.toDegrees(robotToCamera.getRotation().getY()),
            Math.toDegrees(robotToCamera.getRotation().getZ()));
        // Use the robot's gyro (sent every loop) for MegaTag2. The LL4's internal IMU modes
        // (imumode_set 1-4) are worth an experiment but are not the default until measured.
        LimelightHelpers.SetIMUMode(name, 0);
        LimelightHelpers.setLimelightNTDouble(name, "rewind_enable_set", 1);
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSec) {
        LimelightHelpers.SetRobotOrientation(name, yawDegrees, yawRateDegreesPerSec, 0, 0, 0, 0);
    }

    @Override
    public Optional<VisionMeasurement> megaTag2() {
        return toMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name), true);
    }

    @Override
    public Optional<VisionMeasurement> megaTag1() {
        return toMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue(name), false);
    }

    private Optional<VisionMeasurement> toMeasurement(PoseEstimate estimate, boolean megaTag2) {
        if (estimate == null || estimate.tagCount == 0 || estimate.pose == null) {
            return Optional.empty();
        }
        double maxAmbiguity = 0.0;
        if (estimate.rawFiducials != null) {
            for (RawFiducial f : estimate.rawFiducials) {
                if (f != null) {
                    maxAmbiguity = Math.max(maxAmbiguity, f.ambiguity);
                }
            }
        }
        return Optional.of(new VisionMeasurement(
            name,
            estimate.pose,
            estimate.timestampSeconds,
            estimate.tagCount,
            estimate.avgTagDist,
            maxAmbiguity,
            estimate.latency,
            megaTag2));
    }

    @Override
    public CameraHealth health() {
        double now = Timer.getFPGATimestamp();
        double heartbeat = LimelightHelpers.getLimelightNTDouble(name, "hb");
        if (Double.isNaN(lastHeartbeat)) {
            // First reading is a baseline, not evidence of frames: a camera that is not plugged in
            // reads 0 here, and counting that as a change would show it connected at boot.
            lastHeartbeat = heartbeat;
        } else if (heartbeat != lastHeartbeat) {
            lastHeartbeat = heartbeat;
            lastHeartbeatChangeSec = now;
        }
        boolean connected = now - lastHeartbeatChangeSec < kHeartbeatTimeoutSec;
        if (!connected) {
            return CameraHealth.disconnected();
        }
        double[] hw = LimelightHelpers.getLimelightNTDoubleArray(name, "hw");
        return new CameraHealth(
            true,
            hw.length > 3 ? hw[3] : Double.NaN,
            hw.length > 0 ? hw[0] : Double.NaN,
            hw.length > 1 ? hw[1] : Double.NaN,
            hw.length > 2 ? hw[2] : Double.NaN,
            (int) LimelightHelpers.getCurrentPipelineIndex(name));
    }

    @Override
    public void saveRewind(double seconds) {
        rewindCounter++;
        LimelightHelpers.setLimelightNTDoubleArray(name, "capture_rewind",
            new double[] {rewindCounter, Math.min(seconds, kMaxRewindSeconds)});
    }

    @Override
    public void saveSnapshot() {
        snapshotCounter++;
        LimelightHelpers.setLimelightNTDouble(name, "snapshot", snapshotCounter);
    }
}
