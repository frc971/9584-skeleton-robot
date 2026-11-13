package frc.robot.vision;

/**
 * Camera health, logged under Vision/Cameras/&lt;name&gt;/ every loop. NaN means "not reported".
 *
 * @param connected   the camera is producing frames (its heartbeat advanced recently)
 * @param fps         frames per second the camera is processing
 * @param cpuTempC    camera CPU temperature
 * @param cpuUsage    camera CPU usage, percent
 * @param ramPercent  camera RAM usage, percent
 * @param pipeline    active pipeline index
 */
public record CameraHealth(
    boolean connected,
    double fps,
    double cpuTempC,
    double cpuUsage,
    double ramPercent,
    int pipeline) {

    public static CameraHealth disconnected() {
        return new CameraHealth(false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, -1);
    }
}
