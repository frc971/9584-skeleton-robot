package frc.robot.vision;

/**
 * Why a vision estimate was or was not given to the pose estimator. Logged for every estimate
 * (Vision/RejectReasons) so a log shows exactly which rule threw away which estimate, and so the
 * 9584-log-analyzer can count them per match.
 *
 * <p>The names are part of the log format. Renaming one breaks old-log comparisons in the
 * analyzer's history, so add new values rather than renaming existing ones.
 */
public enum RejectReason {
    /** Not rejected: the estimate was applied. */
    ACCEPTED,
    /** The camera saw no tags. */
    NO_TAGS,
    /** Same capture timestamp as the last estimate from this camera: already applied. */
    DUPLICATE,
    /** Robot spinning too fast for a single-tag estimate to be trusted. */
    SPINNING,
    /**
     * MegaTag2 heading disagrees with the gyro. MegaTag2 solves position FROM the gyro heading we
     * send it, so a good estimate always faces the way the gyro does; one that doesn't is a bad
     * solve (2026 mttd q8: field center facing 0 degrees for 20 frames, 5 m pose jump).
     */
    HEADING_MISMATCH,
    /** Outside the field walls (plus a small margin). */
    OFF_FIELD,
    /** Single-tag estimate whose tag ambiguity is too high. */
    AMBIGUOUS,
    /** Tags too far away to be accurate. */
    TOO_FAR,
    /**
     * Single-tag estimate that would move the pose further than any real correction. Multi-tag
     * estimates are never rejected for this, so the robot can always recover from a bad pose.
     */
    LARGE_CORRECTION,
    /**
     * Exactly field center, facing exactly 0 degrees: the 2026 mttd bad solve (repeated for 20
     * frames in q8, 5 m pose jump). HEADING_MISMATCH only catches it when the robot is not facing
     * 0 degrees; facing 0, it used to pass every other rule, multi-tag included. Most likely the
     * Limelight reporting a zero pose in its own center-origin field frame (an inference, not
     * confirmed with Limelight).
     */
    FIELD_CENTER
}
