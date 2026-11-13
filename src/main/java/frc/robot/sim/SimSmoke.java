package frc.robot.sim;

import java.util.Random;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

/**
 * A scripted 20-second "match" in simulation, with no Driver Station and no GUI, that leaves a
 * real .wpilog behind. It exists to check the log format against the 9584-log-analyzer:
 *
 * <pre>
 *   ./gradlew simSmoke                       # writes logs/akit_*.wpilog
 *   python3 ../9584-log-analyzer/tools/match_report.py logs/&lt;that file&gt; --robot ../9584-log-analyzer/robot/2027.toml
 * </pre>
 *
 * <p>It runs the "Example Forward 1m" BLine auto (selected on the dashboard chooser, as a driver
 * would), then drives teleop with the driver controller through the real ManualDriveCommand, and publishes fake
 * Limelight data on the same NetworkTables keys a real Limelight 4 uses, so the whole vision path
 * (LimelightHelpers parsing, filter, weighting, logging) runs. The fake camera includes the 2026
 * mttd q8 bad solve (field center, facing 0 degrees) and wild single-tag estimates, so rejections
 * show up in the log.
 *
 * <p>Off unless the SIM_SMOKE environment variable is set; it can never run on a real robot
 * because Robot only calls it from simulationPeriodic.
 */
public final class SimSmoke {
    public static boolean requested() {
        return System.getenv("SIM_SMOKE") != null;
    }

    private static final String kCamera = "limelight-front";
    /** Selected on the dashboard chooser exactly as a driver would, by its display name. */
    private static final String kAuto = "Example Forward 1m";
    private static final double kAutoStart = 2.0;
    private static final double kAutoEnd = 6.0;
    private static final double kTeleopStart = 7.0;
    private static final double kTeleopEnd = 17.0;
    private static final double kExit = 20.0;

    private final Supplier<Pose2d> truePose;
    private final XboxControllerSim driver = new XboxControllerSim(0);
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable(kCamera);
    private final Random random = new Random(9584);
    private final Timer timer = new Timer();
    private double heartbeat = 0;

    public SimSmoke(Supplier<Pose2d> truePose) {
        this.truePose = truePose;
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setMatchType(MatchType.Qualification);
        DriverStationSim.setMatchNumber(1);
        DriverStationSim.setEventName("SIMSMOKE");
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto Mode").getEntry("selected").setString(kAuto);
        timer.start();
        System.out.println("SimSmoke: scripted match starting");
    }

    public void periodic() {
        double t = timer.get();
        boolean auto = t >= kAutoStart && t < kAutoEnd;
        boolean teleop = t >= kTeleopStart && t < kTeleopEnd;
        DriverStationSim.setAutonomous(auto);
        DriverStationSim.setEnabled(auto || teleop);
        drive(teleop ? t - kTeleopStart : -1);
        DriverStationSim.notifyNewData();
        publishFakeLimelight(t);

        if (t >= kExit) {
            System.out.println("SimSmoke: done, closing log");
            Logger.end();
            System.exit(0);
        }
    }

    /**
     * Turn in place, strafe, stop (the auto already drove forward 1 m). Kept short and in open
     * floor near the blue wall: the simulated field has real structures and a robot driven into one
     * just sits there.
     */
    private void drive(double t) {
        double forward = 0, left = 0, rotate = 0;
        if (t >= 0.5 && t < 3.5) {
            rotate = 0.7;
        } else if (t >= 4 && t < 5) {
            left = -0.6;  // field-centric, toward the right-hand wall, away from the hub
        }
        // ManualDriveCommand negates the sticks (stick up is negative Y on a real controller).
        driver.setLeftY(-forward);
        driver.setLeftX(-left);
        driver.setRightX(-rotate);
    }

    private void publishFakeLimelight(double t) {
        heartbeat++;
        limelight.getEntry("hb").setDouble(heartbeat);
        limelight.getEntry("hw").setDoubleArray(new double[] {48.0, 35.0, 41.0, 60.0});
        limelight.getEntry("getpipe").setDouble(0);

        Pose2d pose = truePose.get();
        double x = pose.getX() + random.nextGaussian() * 0.03;
        double y = pose.getY() + random.nextGaussian() * 0.03;
        double yawDeg = pose.getRotation().getDegrees();
        int tags = 2;
        double dist = 2.5;
        double ambiguity = 0.05;

        double phase = t % 5.0;
        if (phase > 4.0 && phase < 4.4) {
            // 2026 mttd q8 bad solve: exactly field center, facing 0 degrees.
            x = 8.27;
            y = 4.03;
            yawDeg = 0.0;
        } else if (phase > 2.0 && phase < 2.2) {
            // Single-tag estimate 3 m off with the right heading.
            x += 3.0;
            tags = 1;
            dist = 4.0;
        }
        limelight.getEntry("botpose_orb_wpiblue").setDoubleArray(poseArray(x, y, yawDeg, tags, dist, ambiguity));
        limelight.getEntry("botpose_wpiblue").setDoubleArray(poseArray(x, y, yawDeg, tags, dist, ambiguity));
    }

    /** LimelightHelpers layout: x y z roll pitch yaw latency tagCount span dist area, then 7 per tag. */
    private static double[] poseArray(double x, double y, double yawDeg, int tags, double dist, double ambiguity) {
        double[] a = new double[11 + 7 * tags];
        a[0] = x;
        a[1] = y;
        a[5] = yawDeg;
        a[6] = 25.0; // latency ms
        a[7] = tags;
        a[8] = 0.5;
        a[9] = dist;
        a[10] = 0.2;
        for (int i = 0; i < tags; i++) {
            int b = 11 + 7 * i;
            a[b] = i + 1;
            a[b + 4] = dist;
            a[b + 5] = dist;
            a[b + 6] = ambiguity;
        }
        return a;
    }
}
