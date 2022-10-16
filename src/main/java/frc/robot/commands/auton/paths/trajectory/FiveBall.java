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
import frc.robot.sensors.Astra;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * A five ball auto not using trajectories. See the auton sheets to visualize
 * the path.
 */
public class FiveBall extends SequentialCommandGroup implements AutoTrajectoryCommand {
    private ArrayList<AutoPath> autoPaths;

    public FiveBall(ArrayList<AutoPath> autoPaths, Drivetrain drivetrain, Shooter shooter, Intake intake,
            Hopper hopper, Limelight limelight, Astra astra) {
        this.autoPaths = autoPaths;
        addCommands(
                new DrivetrainTrajectoryCommand(autoPaths.get(0).getTrajectory(), drivetrain)
                        .beforeStarting(new InstantCommand(() -> drivetrain
                                .resetOdometry(autoPaths.get(0).getTrajectory().getInitialPose()))),
                new InstantCommand(intake::setDown, intake), // intake down
                new ShootSequence(drivetrain, shooter, limelight, hopper), // shoot 1st ball
                new ParallelRaceGroup( // drive and intake 2nd ball
                        new DrivetrainTrajectoryCommand(autoPaths.get(1).getTrajectory(), drivetrain),
                        new ParallelCommandGroup(
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper),
                                new StartEndCommand(intake::runMotor, intake::stopMotor, intake))),
                new ShootSequence(drivetrain, shooter, limelight, hopper), // shoot 2nd and 3rd ball
                new ParallelRaceGroup(
                        new DrivetrainTrajectoryCommand(autoPaths.get(2).getTrajectory(), drivetrain),
                        new ParallelCommandGroup(
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper),
                                new StartEndCommand(intake::runMotor, intake::stopMotor, intake))), // drive and intake
                                                                                                    // 4th and
                // 5th ball
                new ParallelRaceGroup(
                        new WaitCommand(1.0),
                        new ParallelCommandGroup(
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper),
                                new StartEndCommand(intake::runMotor, intake::stopMotor, intake))), // wait for human
                                                                                                    // player to
                // give 5th ball
                new DrivetrainTrajectoryCommand(autoPaths.get(3).getTrajectory(), drivetrain),
                new ShootSequence(drivetrain, shooter, limelight, hopper), // shoot 4th and 5th ball
                new InstantCommand(intake::setUp, intake)); // intake up
    }

    @Override
    public ArrayList<AutoPath> getAutoPaths() {
        return autoPaths;
    }
}
