package frc.robot.commands.auton.paths.trajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.auton.DrivetrainTrajectoryCommand;
import frc.robot.commands.auton.ShootSequence;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ThreeBall2 extends SequentialCommandGroup {
    public ThreeBall2(Trajectory trajectory, Drivetrain drivetrain, Shooter shooter, Intake intake, Hopper hopper,
            Limelight limelight) {
        addCommands(
                new InstantCommand(intake::setDown, intake),
                new ShootSequence(drivetrain, shooter, limelight, hopper),
                new ParallelRaceGroup(
                        new DrivetrainTrajectoryCommand(trajectory, drivetrain),
                        new StartEndCommand(intake::runMotor, intake::stopMotor, intake)),
                new ShootSequence(drivetrain, shooter, limelight, hopper),
                new InstantCommand(intake::setUp, intake));
    }
}
