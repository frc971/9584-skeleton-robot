package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * Robot-wide constants. Mechanism constants belong in the mechanism's own class, not here.
 */
public final class Constants {
    private Constants() {}

    /** Teleop driving. Used by commands/TeleopDrive.java; see util/StickShaping.java for the curve. */
    public static final class Driving {
        /** Full stick. Open-loop voltage drive, so this scales stick to volts (full stick = 12 V). */
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        /** Full rotation stick: how fast the target heading turns. */
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);

        /**
         * Placeholder. Set it just above the controllers' real stick drift: log an untouched
         * controller and read DriverStation/Joystick0 (typically 0.05 to 0.10).
         */
        public static final double kTranslationDeadband = 0.08;
        public static final double kRotationDeadband = 0.08;
        /** 0 = pure cube (soft near center), 1 = linear. Rotation gets more linear for fine aiming. */
        public static final double kTranslationLinearBlend = 0.3;
        public static final double kRotationLinearBlend = 0.5;

        /**
         * Largest change in commanded field velocity, m/s². Infinity = off. Lower it if the robot
         * tips or browns out on hard starts and stops; tune on the real robot, not in simulation.
         */
        public static final double kMaxTranslationAccelMetersPerSecSq = Double.POSITIVE_INFINITY;

        /** How far the rotation stick may push the target heading ahead of the robot. */
        public static final Angle kHeadingMaxLead = Degrees.of(30);
        /**
         * Heading controller (rad/s of turn per radian of error). Placeholder: 7 is the rotation gain
         * CTRE's examples use for path following. Tune on the robot: raise P until it snaps to a
         * heading crisply, add D if it then overshoots and wobbles. These gains are the turning feel.
         */
        public static final double kHeadingKP = 7.0;
        public static final double kHeadingKD = 0.0;
        /** Heading-controller outputs smaller than this are dropped, so the robot does not buzz on target. */
        public static final AngularVelocity kHeadingRotationalDeadband = kMaxRotationalRate.times(0.005);
    }

    public static final class Field {
        /**
         * This season's AprilTag layout. WPILib's default field changes every season, so this
         * follows the new game with a WPILib update and nothing else. Field size for the vision
         * filter comes from here too, never from a hard-coded number.
         */
        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    public static final class VisionConstants {
        /**
         * One entry per Limelight 4 on the robot. The name must match the camera's hostname
         * ("limelight-front" is http://limelight-front.local:5801).
         *
         * <p>robotToCamera: where the camera's lens is relative to the center of the robot on the
         * floor, measured from CAD. x forward, y LEFT, z up, meters; roll, pitch, yaw in radians.
         * Pitch and yaw signs are easy to get backwards: after deploying, check each camera in
         * its web UI's 3D view, and do the known-pose test in docs/vision.md. The zeros below are
         * placeholders: an unmeasured camera gives confident, wrong poses.
         *
         * <p>A camera listed here but not plugged in raises a red dashboard alert. Delete entries
         * for cameras the robot does not have.
         */
        public record Camera(String name, Transform3d robotToCamera, int pipeline) {}

        public static final List<Camera> kCameras = List.of(
            new Camera("limelight-front", new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)), 0),
            new Camera("limelight-back", new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.PI)), 0),
            new Camera("limelight-left", new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.PI / 2)), 0)
        );
    }
}
