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

public class Intake extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.Intake.CANIDs.MOTOR,
            MotorType.kBrushless);
    private DoubleSolenoid solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            Constants.Intake.Solenoids.INTAKE_CHANNEL_1,
            Constants.Intake.Solenoids.INTAKE_CHANNEL_2);

    public Intake() {
        motor.setInverted(true);
        LoggingManager.getInstance().addGroup("Intake", new LogGroup(
                new DeviceLogger<CANSparkMax>(motor, "Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(motor)),
                new DeviceLogger<DoubleSolenoid>(solenoid, "Gatekeepers",
                        LogProfileBuilder.buildDoubleSolenoidLogItems(solenoid))));
    }

    /**
     * Sets the intake to the up position.
     */
    public void setUp() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Sets the intake to the down position.
     */
    public void setDown() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Runs the intake motors.
     */
    public void runMotor() {
        motor.set(1);
    }

    /**
     * Runs the intake motors in reverse.
     */
    public void reverseMotor() {
        motor.set(-1);
    }

    /**
     * Stops the intake motors.
     */
    public void stopMotor() {
        motor.set(0);
    }
}
