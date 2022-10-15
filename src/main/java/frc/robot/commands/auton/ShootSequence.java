package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.RunShooterLockedSpeed;
import frc.robot.commands.TurnToGoal;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/**
 * Defines the sequence of steps we use to shoot a ball. Could be mapped to a
 * controller button to automate the shooting sequence.
 * 
 * @author dr
 */
public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(Drivetrain drivetrain, Shooter shooter, Limelight limelight, Hopper hopper) {
        addCommands(
                new InstantCommand(limelight::setVisionMode),
                new TurnToGoal(drivetrain, limelight).withTimeout(1.5),
                new ParallelRaceGroup(
                        // new RunShooterSetSpeed(33.0, shooter),
                        new RunShooterLockedSpeed(shooter, limelight),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(shooter::atSetpoint),
                                new InstantCommand(hopper::gatekeepersOpen, hopper),
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper).withTimeout(1.4),
                                new InstantCommand(hopper::gatekeepersClosed, hopper))),
                new InstantCommand(limelight::setDriverAssistMode));

    }
}
