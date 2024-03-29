package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.enums.LogType;
import com.techhounds.houndutil.houndlog.loggers.DeviceLogger;
import com.techhounds.houndutil.houndlog.loggers.SingleItemLogger;
import frc.robot.Constants;

/**
 * The shooter subsystem, created as a PID subsystem. Contains the motors that
 * control the flywheel.
 */
public class Shooter extends PIDSubsystem {
    /** The primary motor of the shooter. */
    private CANSparkMax primaryMotor = new CANSparkMax(Constants.Shooter.CANIDs.PRIMARY,
            MotorType.kBrushless);
    /** The secondary motor of the shooter. */
    private CANSparkMax secondaryMotor = new CANSparkMax(Constants.Shooter.CANIDs.SECONDARY,
            MotorType.kBrushless);
    /** The group that controls both shooter motors. */
    private MotorControllerGroup motors = new MotorControllerGroup(primaryMotor, secondaryMotor);

    /** The feed-forward controller that runs the shooter motors. */
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Shooter.kS, Constants.Shooter.kV,
            Constants.Shooter.kA);

    /**
     * Initializes the shooter.
     */
    public Shooter() {
        super(new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD));

        getController().setTolerance(1.0);

        // to convert the measurements from RPMs to rot/s (thanks REV robotics)
        primaryMotor.getEncoder().setVelocityConversionFactor(1.0 / 60.0);
        primaryMotor.setInverted(true);

        LoggingManager.getInstance().addGroup("Shooter", new LogGroup(
                new DeviceLogger<CANSparkMax>(primaryMotor, "Primary Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(primaryMotor)),
                new DeviceLogger<CANSparkMax>(secondaryMotor, "Secondary Motor",
                        LogProfileBuilder.buildCANSparkMaxLogItems(secondaryMotor)),
                new SingleItemLogger<Double>(LogType.NUMBER, "PID Setpoint", this::getSetpoint),
                new SingleItemLogger<Double>(LogType.NUMBER, "PID Measurement", this::getMeasurement),
                new SingleItemLogger<Double>(LogType.NUMBER, "Velocity", this::getVelocity)));
    }

    /**
     * Gets the measurement for the PID loop. This is used through the subsystem's
     * {@code enable} method.
     */
    @Override
    public double getMeasurement() {
        return getVelocity();
    }

    /**
     * The consumer that uses the output of the PID calculation. This adds a
     * feedforward value to the voltage.
     */
    @Override

    public void useOutput(double output, double setpoint) {
        System.out.println("PID" + output);
        System.out.println(feedforward.calculate(setpoint));
        setSpeedVolts(output + feedforward.calculate(setpoint));
    }

    /**
     * Sets the speed of the motors.
     * 
     * @param speed the speed of the motors (-1 to 1).
     */
    public void setSpeed(double speed) {
        motors.set(speed);
    }

    /**
     * Sets the speed of the motors in volts. This must be called regularly in order
     * for voltage compensation to work, a fundamental component of feed-forward
     * control.
     * 
     * @param volts the speed of the motors in volts (-12v to 12v)
     */
    public void setSpeedVolts(double volts) {
        motors.setVoltage(volts);
    }

    /**
     * Resets the shooter.
     */
    public void resetEncoders() {
        primaryMotor.getEncoder().setPosition(0);
    }

    /**
     * Gets the current velocity of the shooter.
     * 
     * @return The current speed of the shooter, in rpm.
     */
    public double getVelocity() {
        return primaryMotor.getEncoder().getVelocity();
    }

    /**
     * Stops the shooter.
     */
    public void stop() {
        setSpeed(0);
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }
}
