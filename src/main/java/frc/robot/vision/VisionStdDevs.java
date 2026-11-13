package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * How much the pose estimator should trust one vision estimate, as standard deviations
 * [x meters, y meters, heading radians]. Smaller = trusted more.
 *
 * <p>Shape: error from a tag grows roughly with the square of its distance, so trust falls off as
 * {@code base + scale * distance^2}. One tag is less reliable than several, so single-tag estimates
 * are multiplied up and many-tag estimates down. This is the 2026 robot's curve, kept so last
 * season's logs remain comparable. The constants have never been fitted to data: the
 * known-pose test in docs/vision.md is how to fit them.
 *
 * <p>Heading: MegaTag2 takes its heading FROM the gyro, so feeding that heading back into the
 * estimator would let the gyro correct itself in a loop. Heading trust is therefore effectively
 * zero for MegaTag2 estimates.
 *
 * <p>Known problem from 2026 (analyzer GEMINI.md): single-tag estimates were trusted at 5-10 cm,
 * which is too much. {@link Config#singleTagMultiplier} is the knob for that.
 */
public final class VisionStdDevs {
    /** Effectively infinite: the estimator ignores this axis. */
    public static final double kIgnore = 1e6;

    public record Config(
        double baseMeters,
        double scalePerMeterSquared,
        double singleTagMultiplier,
        int manyTagCount,
        double manyTagMultiplier) {

        /** The 2026 robot's values (VisionSubsystem.getVisionStdDevsForEstimate). */
        public static Config defaults() {
            return new Config(0.02, 0.02, 2.0, 5, 0.7);
        }
    }

    private final Config config;

    public VisionStdDevs(Config config) {
        this.config = config;
    }

    /** Standard deviation in meters for x and y. */
    public double xyMeters(VisionMeasurement m) {
        double d = m.avgTagDistMeters();
        double xy = config.baseMeters() + config.scalePerMeterSquared() * d * d;
        if (m.tagCount() <= 1) {
            xy *= config.singleTagMultiplier();
        } else if (m.tagCount() >= config.manyTagCount()) {
            xy *= config.manyTagMultiplier();
        }
        return xy;
    }

    /** Standard deviation in radians for heading. */
    public double headingRadians(VisionMeasurement m) {
        return m.megaTag2() ? kIgnore : xyMeters(m) * 10.0;
    }

    public Matrix<N3, N1> forMeasurement(VisionMeasurement m) {
        double xy = xyMeters(m);
        return VecBuilder.fill(xy, xy, headingRadians(m));
    }
}
