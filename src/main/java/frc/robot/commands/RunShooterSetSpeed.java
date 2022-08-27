package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * Runs the shooter. Now that the shooter is a PIDSubsystem, all that needs to
 * happen is to run the {@code enable()} method.
 */
public class RunShooterSetSpeed extends CommandBase {
    private double rpm;
    private Shooter shooter;

    public RunShooterSetSpeed(double rpm, Shooter shooter) {
        this.rpm = rpm;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSetpoint(rpm);
        shooter.enable();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
