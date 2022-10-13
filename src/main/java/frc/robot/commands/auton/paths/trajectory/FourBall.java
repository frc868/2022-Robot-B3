package frc.robot.commands.auton.paths.trajectory;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auton.DrivetrainTrajectoryCommand;
import frc.robot.commands.auton.ShootSequence;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FourBall extends SequentialCommandGroup {
    public FourBall(HashMap<String, Trajectory> trajectories, Drivetrain drivetrain, Shooter shooter, Intake intake,
            Hopper hopper,
            Limelight limelight) {
        addCommands(
                new InstantCommand(intake::setDown, intake),
                new ParallelRaceGroup(
                        new DrivetrainTrajectoryCommand(trajectories.get("4Ball.To2"), drivetrain),
                        new StartEndCommand(intake::runMotor, intake::stopMotor, intake)),
                new ShootSequence(drivetrain, shooter, limelight, hopper),
                new ParallelRaceGroup(
                        new DrivetrainTrajectoryCommand(trajectories.get("4Ball.To3and4"), drivetrain),
                        new StartEndCommand(intake::runMotor, intake::stopMotor, intake)),
                new ParallelRaceGroup(
                        new WaitCommand(3),
                        new StartEndCommand(intake::runMotor, intake::stopMotor, intake)),
                new DrivetrainTrajectoryCommand(trajectories.get("4Ball.ToGoal"), drivetrain),
                new ShootSequence(drivetrain, shooter, limelight, hopper),
                new InstantCommand(intake::setUp, intake));
    }
}
