package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class GyroDriftCheckTest {
    private static final double kFrame = 0.02;

    /** The first version warned "87 deg off" on a single bad frame. */
    @Test
    void oneBadFrameDoesNotMoveTheAnswer() {
        GyroDriftCheck check = new GyroDriftCheck();
        for (int i = 0; i < 20; i++) {
            check.add(i * kFrame, i == 7 ? 87.0 : 0.5);
        }
        assertEquals(0.5, check.driftDegrees(20 * kFrame).getAsDouble(), 1e-9);
    }

    @Test
    void steadyOffsetIsReported() {
        GyroDriftCheck check = new GyroDriftCheck();
        for (int i = 0; i < 20; i++) {
            check.add(i * kFrame, 8.0 + (i % 2 == 0 ? 0.3 : -0.3));
        }
        assertEquals(8.0, check.driftDegrees(20 * kFrame).getAsDouble(), 1e-9);
    }

    @Test
    void tooFewFramesGiveNoAnswer() {
        GyroDriftCheck check = new GyroDriftCheck();
        for (int i = 0; i < GyroDriftCheck.kMinSamples - 1; i++) {
            check.add(i * kFrame, 30.0);
        }
        assertTrue(check.driftDegrees(0.5).isEmpty());
    }

    @Test
    void oldFramesExpire() {
        GyroDriftCheck check = new GyroDriftCheck();
        for (int i = 0; i < 20; i++) {
            check.add(i * kFrame, 30.0);
        }
        assertTrue(check.driftDegrees(20 * kFrame + GyroDriftCheck.kWindowSec + 0.1).isEmpty());
    }
}
