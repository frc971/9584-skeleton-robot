package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.SimSmoke;

/**
 * Robot lifecycle and logging setup. Robot behavior belongs in {@link RobotContainer}.
 *
 * <p>Logs are AdvantageKit .wpilog files, read by AdvantageScope and by the 9584-log-analyzer
 * repo. What gets logged, and the names the analyzer depends on, are in docs/logging.md.
 */
public class Robot extends LoggedRobot {
    /**
     * A USB stick in the roboRIO shows up here. Logs go only to the stick, never to the roboRIO's
     * own storage, which is small and fills up in a few practices.
     * SystemCore: the mount point is different; see docs/porting-2027.md.
     */
    private static final String kUsbLogPath = "/media/sda1/";
    private final Alert noUsbLogAlert = new Alert(
        "No USB stick in the roboRIO. NO LOG IS BEING RECORDED this session.", AlertType.kError);

    private final RobotContainer robotContainer;
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    private Command autonomousCommand;

    public Robot() {
        configureLogging();
        robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));
    }

    private void configureLogging() {
        Logger.recordMetadata("ProjectName", "9584-skeleton-robot");
        Logger.recordMetadata("Robot", RobotBase.isReal() ? "Real" : "Simulation");

        if (RobotBase.isReal()) {
            Path usb = Path.of(kUsbLogPath);
            boolean usbPresent = Files.isDirectory(usb) && Files.isWritable(usb);
            noUsbLogAlert.set(!usbPresent);
            if (usbPresent) {
                Logger.addDataReceiver(new WPILOGWriter(kUsbLogPath));
            } else {
                DriverStation.reportError("No USB stick in the roboRIO; no log will be recorded.", false);
            }
        } else {
            try {
                Files.createDirectories(Path.of("logs"));
            } catch (IOException ex) {
                DriverStation.reportError("Failed to create logs directory: " + ex.getMessage(), false);
            }
            Logger.addDataReceiver(new WPILOGWriter("logs/"));
        }
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        if (RobotBase.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        logPowerAndCan();
    }

    /** Read by the analyzer's power check (battery sag vs current, brownouts). */
    private void logPowerAndCan() {
        Logger.recordOutput("Power/TotalCurrent", pdh.getTotalCurrent());
        Logger.recordOutput("Power/Voltage", pdh.getVoltage());

        var canStatus = RobotController.getCANStatus();
        Logger.recordOutput("CAN/BusUtilization", canStatus.percentBusUtilization);
        Logger.recordOutput("CAN/BusOffCount", canStatus.busOffCount);
        Logger.recordOutput("CAN/TxFullCount", canStatus.txFullCount);
        Logger.recordOutput("CAN/ReceiveErrorCount", canStatus.receiveErrorCount);
        Logger.recordOutput("CAN/TransmitErrorCount", canStatus.transmitErrorCount);
    }

    @Override
    public void disabledInit() {
        robotContainer.onDisable();
    }

    @Override
    public void disabledPeriodic() {
        robotContainer.whileDisabled();
    }

    @Override
    public void autonomousInit() {
        robotContainer.resetPoseForAuto();
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private SimSmoke simSmoke;

    @Override
    public void simulationInit() {
        if (SimSmoke.requested()) {
            simSmoke = new SimSmoke(robotContainer::getPose);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (simSmoke != null) {
            simSmoke.periodic();
        }
    }
}
