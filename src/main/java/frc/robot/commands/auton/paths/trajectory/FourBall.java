package frc.robot.commands.auton.paths.trajectory;

import java.util.ArrayList;
import com.techhounds.houndutil.houndlib.auto.AutoPath;
import com.techhounds.houndutil.houndlib.auto.AutoTrajectoryCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class FourBall extends SequentialCommandGroup implements AutoTrajectoryCommand {
    private ArrayList<AutoPath> autoPaths;

    public FourBall(ArrayList<AutoPath> autoPaths, Drivetrain drivetrain, Shooter shooter, Intake intake,
            Hopper hopper,
            Limelight limelight) {
        this.autoPaths = autoPaths;
        addCommands(
                new InstantCommand(intake::setDown, intake),
                new ParallelRaceGroup(
                        new DrivetrainTrajectoryCommand(autoPaths.get(0).getTrajectory(), drivetrain),
                        new ParallelCommandGroup(
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper),
                                new StartEndCommand(intake::runMotor, intake::stopMotor, intake))),
                new ShootSequence(drivetrain, shooter, limelight, hopper),
                new ParallelRaceGroup(
                        new DrivetrainTrajectoryCommand(autoPaths.get(1).getTrajectory(), drivetrain),
                        new ParallelCommandGroup(
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper),
                                new StartEndCommand(intake::runMotor, intake::stopMotor, intake))),
                new ParallelRaceGroup(
                        new WaitCommand(3),
                        new ParallelCommandGroup(
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper),
                                new StartEndCommand(intake::runMotor, intake::stopMotor, intake))),
                new DrivetrainTrajectoryCommand(autoPaths.get(2).getTrajectory(), drivetrain),
                new ShootSequence(drivetrain, shooter, limelight, hopper),
                new InstantCommand(intake::setUp, intake));
    }

    @Override
    public ArrayList<AutoPath> getAutoPaths() {
        return autoPaths;
    }
}
