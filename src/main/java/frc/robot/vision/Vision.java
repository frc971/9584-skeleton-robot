package frc.robot.vision;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * AprilTag localization: reads every camera, decides which estimates to trust, feeds the pose
 * estimator, and logs everything needed to diagnose it afterwards.
 *
 * <p>Each loop, for each camera:
 * <ol>
 *   <li>send it the robot heading (the ONLY place heading is sent; see {@link VisionCamera});</li>
 *   <li>read its MegaTag2 estimate; skip it if it is the same capture as last loop;</li>
 *   <li>run {@link VisionFilter}; if accepted, weight it with {@link VisionStdDevs} and hand it to
 *       the pose estimator;</li>
 *   <li>log the estimate, the verdict and the reason, accepted or not.</li>
 * </ol>
 *
 * <p>Also: checks the gyro against MegaTag1 headings while disabled (gyro drift is invisible to
 * MegaTag2, which trusts the gyro), saves Limelight Rewind video at the end of every FMS match, and
 * saves a still frame when an estimate is rejected for a reason worth looking at.
 *
 * <p>The log signal names are a contract with the 9584-log-analyzer repo; see docs/logging.md
 * before renaming anything.
 */
public class Vision extends SubsystemBase {

    /** What vision needs from the drivetrain. Kept as an interface so this class has no CTRE code in it. */
    public interface PoseEstimator {
        /** The fused pose right now. */
        Pose2d pose();

        /** Robot rotation rate, rad/s, counterclockwise positive. */
        double yawRateRadPerSec();

        /**
         * @param timestampSeconds capture time on the FPGA clock; the drivetrain converts it
         */
        void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    private static final String kLog = "Vision/";
    /** MegaTag1 heading vs gyro, while disabled with at least this many tags, raises an alert. */
    private static final int kGyroCheckMinTags = 2;
    private static final double kGyroDriftAlertDegrees = 5.0;
    /** At most one automatic snapshot per this many seconds, so the camera's disk does not fill. */
    private static final double kSnapshotIntervalSec = 2.0;

    private final List<VisionCamera> cameras;
    private final VisionFilter filter;
    private final VisionStdDevs stdDevs;
    private final PoseEstimator estimator;

    private final Map<String, Double> lastTimestamps = new HashMap<>();
    private final EnumMap<RejectReason, Integer> reasonCounts = new EnumMap<>(RejectReason.class);
    private final Alert gyroDriftAlert = new Alert("", AlertType.kWarning);
    private final List<Alert> cameraAlerts = new ArrayList<>();

    private boolean wasEnabled = false;
    private boolean wasTeleop = false;
    private int rewindSaves = 0;
    private int snapshots = 0;
    private double lastSnapshotSec = Double.NEGATIVE_INFINITY;

    public Vision(List<VisionCamera> cameras, VisionFilter filter, VisionStdDevs stdDevs,
        PoseEstimator estimator) {
        this.cameras = List.copyOf(cameras);
        this.filter = filter;
        this.stdDevs = stdDevs;
        this.estimator = estimator;
        for (RejectReason r : RejectReason.values()) {
            reasonCounts.put(r, 0);
        }
        for (VisionCamera camera : this.cameras) {
            cameraAlerts.add(new Alert(camera.name() + " is not sending frames", AlertType.kError));
        }
        Logger.recordOutput(kLog + "CameraNames",
            this.cameras.stream().map(VisionCamera::name).toArray(String[]::new));
    }

    @Override
    public void periodic() {
        Pose2d robotPose = estimator.pose();
        double yawRate = estimator.yawRateRadPerSec();
        double yawDegrees = robotPose.getRotation().getDegrees();
        Logger.recordOutput(kLog + "GyroYawDegrees", yawDegrees);
        Logger.recordOutput(kLog + "OmegaRadPerSec", yawRate);

        List<VisionMeasurement> estimates = new ArrayList<>();
        List<VisionFilter.Decision> decisions = new ArrayList<>();
        List<Matrix<N3, N1>> weights = new ArrayList<>();

        for (int i = 0; i < cameras.size(); i++) {
            VisionCamera camera = cameras.get(i);
            camera.setRobotOrientation(yawDegrees, Math.toDegrees(yawRate));

            CameraHealth health = camera.health();
            logHealth(camera.name(), health);
            cameraAlerts.get(i).set(!health.connected());

            Optional<VisionMeasurement> maybe = camera.megaTag2();
            if (maybe.isEmpty()) {
                continue;
            }
            VisionMeasurement m = maybe.get();
            double last = lastTimestamps.getOrDefault(camera.name(), Double.NaN);
            VisionFilter.Decision d = filter.evaluate(m, robotPose, yawRate, last);
            if (d.reason() == RejectReason.DUPLICATE) {
                // Same image as last loop: already handled, and logging it again would make one
                // estimate look like many. Counted only.
                count(RejectReason.DUPLICATE);
                continue;
            }
            lastTimestamps.put(camera.name(), m.timestampSeconds());
            Matrix<N3, N1> w = stdDevs.forMeasurement(m);
            if (d.accepted()) {
                estimator.addVisionMeasurement(m.pose(), m.timestampSeconds(), w);
            } else {
                maybeSnapshot(camera, d.reason());
            }
            count(d.reason());
            estimates.add(m);
            decisions.add(d);
            weights.add(w);
        }

        logEstimates(estimates, decisions, weights);
        checkGyroWhileDisabled(robotPose);
        saveRewindAtMatchEnd();
    }

    private void count(RejectReason reason) {
        int n = reasonCounts.merge(reason, 1, Integer::sum);
        Logger.recordOutput(kLog + "ReasonCounts/" + reason.name(), n);
    }

    /**
     * One array entry per NEW estimate this loop, every array the same length and order.
     * The first seven names are the 2026 robot's, unchanged, so the analyzer's existing vision
     * check reads this robot's logs with no edits.
     */
    private void logEstimates(List<VisionMeasurement> estimates, List<VisionFilter.Decision> decisions,
        List<Matrix<N3, N1>> weights) {
        int n = estimates.size();
        Pose2d[] poses = new Pose2d[n];
        double[] stdXY = new double[n];
        double[] stdTheta = new double[n];
        double[] tagCounts = new double[n];
        double[] tagDists = new double[n];
        double[] latencies = new double[n];
        String[] camerasOut = new String[n];
        double[] timestamps = new double[n];
        double[] ambiguities = new double[n];
        boolean[] accepted = new boolean[n];
        String[] reasons = new String[n];
        double[] residualMeters = new double[n];
        double[] residualDegrees = new double[n];
        List<Pose2d> acceptedPoses = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            VisionMeasurement m = estimates.get(i);
            VisionFilter.Decision d = decisions.get(i);
            poses[i] = m.pose();
            stdXY[i] = weights.get(i).get(0, 0);
            stdTheta[i] = weights.get(i).get(2, 0);
            tagCounts[i] = m.tagCount();
            tagDists[i] = m.avgTagDistMeters();
            latencies[i] = m.latencyMillis();
            camerasOut[i] = m.camera();
            timestamps[i] = m.timestampSeconds();
            ambiguities[i] = m.maxAmbiguity();
            accepted[i] = d.accepted();
            reasons[i] = d.reason().name();
            residualMeters[i] = d.residualMeters();
            residualDegrees[i] = d.residualDegrees();
            if (d.accepted()) {
                acceptedPoses.add(m.pose());
            }
        }

        // 2026 names (read by the analyzer's tools/checks/vision.py).
        Logger.recordOutput(kLog + "EstimateCount", n);
        Logger.recordOutput(kLog + "Poses", poses);
        Logger.recordOutput(kLog + "StdDevsXY", stdXY);
        Logger.recordOutput(kLog + "StdDevsTheta", stdTheta);
        Logger.recordOutput(kLog + "TagCounts", tagCounts);
        Logger.recordOutput(kLog + "AvgTagDists", tagDists);
        Logger.recordOutput(kLog + "Latencies", latencies);
        // New for 2027 (the analyzer's GEMINI.md "logging to add next season").
        Logger.recordOutput(kLog + "EstimateCameras", camerasOut);
        Logger.recordOutput(kLog + "CaptureTimestamps", timestamps);
        Logger.recordOutput(kLog + "Ambiguities", ambiguities);
        Logger.recordOutput(kLog + "Accepted", accepted);
        Logger.recordOutput(kLog + "RejectReasons", reasons);
        Logger.recordOutput(kLog + "ResidualMeters", residualMeters);
        Logger.recordOutput(kLog + "ResidualDegrees", residualDegrees);
        Logger.recordOutput(kLog + "AcceptedPoses", acceptedPoses.toArray(Pose2d[]::new));
    }

    private void logHealth(String name, CameraHealth h) {
        String p = kLog + "Cameras/" + name + "/";
        Logger.recordOutput(p + "Connected", h.connected());
        Logger.recordOutput(p + "Fps", h.fps());
        Logger.recordOutput(p + "CpuTempC", h.cpuTempC());
        Logger.recordOutput(p + "CpuUsage", h.cpuUsage());
        Logger.recordOutput(p + "RamPercent", h.ramPercent());
        Logger.recordOutput(p + "Pipeline", h.pipeline());
    }

    /**
     * MegaTag2 trusts the gyro completely, so a drifted gyro produces confident, wrong positions
     * that no filter rule can catch. While disabled, compare the gyro to the heading MegaTag1
     * solves from tags alone. This only reports; it never changes the heading, because resetting
     * the gyro from vision is a decision a person should make (pit, before the match).
     */
    private void checkGyroWhileDisabled(Pose2d robotPose) {
        if (!DriverStation.isDisabled()) {
            return;
        }
        double worst = Double.NaN;
        for (VisionCamera camera : cameras) {
            Optional<VisionMeasurement> mt1 = camera.megaTag1();
            if (mt1.isEmpty() || mt1.get().tagCount() < kGyroCheckMinTags) {
                continue;
            }
            double err = mt1.get().pose().getRotation().minus(robotPose.getRotation()).getDegrees();
            Logger.recordOutput(kLog + "Cameras/" + camera.name() + "/MegaTag1HeadingErrorDeg", err);
            if (Double.isNaN(worst) || Math.abs(err) > Math.abs(worst)) {
                worst = err;
            }
        }
        if (!Double.isNaN(worst)) {
            Logger.recordOutput(kLog + "GyroDriftDegrees", worst);
            gyroDriftAlert.setText(String.format(
                "Gyro heading is %.1f deg off what the tags say. Re-zero heading before the match.", worst));
            gyroDriftAlert.set(Math.abs(worst) > kGyroDriftAlertDegrees);
        }
    }

    /** Saves the whole match's Rewind video on every camera when an FMS match ends. */
    private void saveRewindAtMatchEnd() {
        boolean enabled = DriverStation.isEnabled();
        if (wasEnabled && !enabled && wasTeleop && DriverStation.isFMSAttached()) {
            saveRewind(LimelightCamera.kMaxRewindSeconds, "match end");
        }
        wasEnabled = enabled;
        if (enabled) {
            wasTeleop = DriverStation.isTeleop();
        }
    }

    /** Save the last {@code seconds} of video on every camera. */
    public void saveRewind(double seconds, String why) {
        for (VisionCamera camera : cameras) {
            camera.saveRewind(seconds);
        }
        rewindSaves++;
        Logger.recordOutput(kLog + "Rewind/SaveCount", rewindSaves);
        Logger.recordOutput(kLog + "Rewind/LastReason", why);
        // Printed so it appears in the log's Console and in the analyzer's timeline.
        System.out.println("Vision: saved " + (int) seconds + " s of Rewind video (" + why + ")");
    }

    /** For a driver/operator button: "that looked wrong, keep the video". Works while disabled. */
    public Command saveRewindCommand() {
        return Commands.runOnce(() -> saveRewind(30.0, "button")).ignoringDisable(true)
            .withName("Save Rewind");
    }

    private void maybeSnapshot(VisionCamera camera, RejectReason reason) {
        boolean interesting = reason == RejectReason.HEADING_MISMATCH
            || reason == RejectReason.LARGE_CORRECTION
            || reason == RejectReason.OFF_FIELD;
        double now = Timer.getFPGATimestamp();
        if (!interesting || !DriverStation.isEnabled() || now - lastSnapshotSec < kSnapshotIntervalSec) {
            return;
        }
        lastSnapshotSec = now;
        camera.saveSnapshot();
        snapshots++;
        Logger.recordOutput(kLog + "SnapshotCount", snapshots);
        Logger.recordOutput(kLog + "LastSnapshot", camera.name() + " " + reason.name());
    }
}
