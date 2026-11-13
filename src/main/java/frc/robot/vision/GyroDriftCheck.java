package frc.robot.vision;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.OptionalDouble;

/**
 * Is the gyro heading off from the heading the AprilTags give? Used while disabled, where it can
 * still be fixed by re-zeroing before the match.
 *
 * <p>MegaTag2 trusts the gyro completely, so a drifted gyro gives confident, wrong positions that
 * no filter rule can see. MegaTag1 solves heading from the tags alone, so it is an independent
 * check, but any single MegaTag1 frame can be a bad solve. So the answer is the MEDIAN error over
 * the last {@link #kWindowSec} seconds, and there is no answer until {@link #kMinSamples} frames
 * have been seen. One bad frame cannot raise a "re-zero" warning; a real, steady offset still does.
 * (The first version used each frame alone and warned 87 degrees off on one field-center bad solve.)
 */
public final class GyroDriftCheck {
    static final double kWindowSec = 1.0;
    static final int kMinSamples = 10;

    private record Sample(double timeSec, double errorDeg) {}

    private final Deque<Sample> samples = new ArrayDeque<>();

    /** Add one MegaTag1 heading error (tags minus gyro, degrees, already wrapped to +/-180). */
    public void add(double timeSec, double errorDeg) {
        samples.addLast(new Sample(timeSec, errorDeg));
        prune(timeSec);
    }

    /** Forget everything, e.g. when the robot is enabled or the heading is re-zeroed. */
    public void clear() {
        samples.clear();
    }

    /** Median error over the window, or empty if there are too few recent frames to say. */
    public OptionalDouble driftDegrees(double nowSec) {
        prune(nowSec);
        if (samples.size() < kMinSamples) {
            return OptionalDouble.empty();
        }
        double[] errors = samples.stream().mapToDouble(Sample::errorDeg).sorted().toArray();
        int n = errors.length;
        double median = n % 2 == 1 ? errors[n / 2] : (errors[n / 2 - 1] + errors[n / 2]) / 2.0;
        return OptionalDouble.of(median);
    }

    int sampleCount() {
        return samples.size();
    }

    private void prune(double nowSec) {
        while (!samples.isEmpty() && nowSec - samples.peekFirst().timeSec() > kWindowSec) {
            samples.removeFirst();
        }
    }

    @Override
    public String toString() {
        return "GyroDriftCheck" + Arrays.toString(samples.stream().mapToDouble(Sample::errorDeg).toArray());
    }
}
