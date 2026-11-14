package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;

class HeadingTargetTest {
    private static final double kMaxLead = Math.toRadians(30);

    @Test
    void startsFromRobotHeadingAndHoldsWithoutStick() {
        HeadingTarget target = new HeadingTarget(kMaxLead);
        Rotation2d robot = Rotation2d.fromDegrees(40);
        assertEquals(40.0, target.update(robot, 0.0, 0.02).getDegrees(), 1e-9);
        // Robot gets pushed; the target does not follow.
        assertEquals(40.0, target.update(Rotation2d.fromDegrees(55), 0.0, 0.02).getDegrees(), 1e-9);
    }

    @Test
    void stickMovesTarget() {
        HeadingTarget target = new HeadingTarget(kMaxLead);
        Rotation2d robot = Rotation2d.kZero;
        target.update(robot, 0.0, 0.02);
        assertEquals(Math.toDegrees(Math.PI * 0.02), target.update(robot, Math.PI, 0.02).getDegrees(), 1e-9);
    }

    @Test
    void stickCannotRunAheadOfABlockedRobot() {
        HeadingTarget target = new HeadingTarget(kMaxLead);
        Rotation2d blocked = Rotation2d.fromDegrees(10);
        for (int i = 0; i < 200; i++) {
            target.update(blocked, 2 * Math.PI, 0.02);
        }
        assertEquals(40.0, target.get().orElseThrow().getDegrees(), 1e-9);
    }

    @Test
    void leadWrapsAcross180() {
        HeadingTarget target = new HeadingTarget(kMaxLead);
        Rotation2d robot = Rotation2d.fromDegrees(170);
        target.update(robot, 0.0, 0.02);
        Rotation2d moved = target.update(robot, Math.toRadians(1000), 0.02); // asks for +20°
        assertEquals(-170.0, moved.getDegrees(), 1e-9);
    }

    @Test
    void setIsNotLimitedAndClearForgets() {
        HeadingTarget target = new HeadingTarget(kMaxLead);
        target.set(Rotation2d.fromDegrees(90));
        assertEquals(90.0, target.update(Rotation2d.kZero, 0.0, 0.02).getDegrees(), 1e-9);
        target.clear();
        assertTrue(target.get().isEmpty());
        assertEquals(-30.0, target.update(Rotation2d.fromDegrees(-30), 0.0, 0.02).getDegrees(), 1e-9);
    }
}
