package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import frc.robot.Constants;

/**
 * The hopper subsystem, includes the hopper motors and the gatekeepers.
 * 
 * @author dr
 */
public class Hopper extends SubsystemBase {
    /** The motor that drives the hopper belt. */
    private CANSparkMax motor = new CANSparkMax(Constants.Hopper.CANIDs.MOTOR, MotorType.kBrushless);

    /** The solenoid that controls the gatekeepers. */
    private DoubleSolenoid gatekeepers = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.Hopper.Solenoids.GATEKEEPER_CHANNEL_2,
            Constants.Hopper.Solenoids.GATEKEEPER_CHANNEL_1);

    /**
     * Constructs the hopper object.
     */
    public Hopper() {

        LoggingManager.getInstance().addGroup("Hopper", new LogGroup(
                new DeviceLogger<CANSparkMax>(motor, "Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(motor)),
                new DeviceLogger<DoubleSolenoid>(gatekeepers, "Gatekeepers",
                        LogProfileBuilder.buildDoubleSolenoidLogItems(gatekeepers))));
    }

    /**
     * Sets the gatekeepers to the "in" position (non-blocking).
     */
    public void gatekeepersOpen() {
        gatekeepers.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sets the gatekeepers to the "out" position (blocking).
     */
    public void gatekeepersClosed() {
        gatekeepers.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Toggles the gatekeepers.
     */
    public void toggleGatekeepers() {
        gatekeepers.toggle();
    }

    /**
     * Runs the hopper motor at full power.
     */
    public void runMotor() {
        motor.set(1);
    }

    /**
     * Runs the hopper motor in reverse at full power.
     */
    public void reverseMotor() {
        motor.set(-1);
    }

    /**
     * Stops the motor.
     */
    public void stopMotor() {
        motor.set(0);
    }
}
