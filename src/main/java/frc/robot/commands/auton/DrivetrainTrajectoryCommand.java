package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives the drivetrain using a RAMSETE controller. Accepts a trajectory to
 * follow as input.
 * 
 * @author dr
 */
public class DrivetrainTrajectoryCommand extends RamseteCommand {
    public Trajectory trajectory;
    public Drivetrain drivetrain;

    public DrivetrainTrajectoryCommand(Trajectory trajectory, Drivetrain drivetrain) {
        super(
                trajectory,
                drivetrain::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(Constants.Drivetrain.PID.kS, Constants.Drivetrain.PID.kV,
                        Constants.Drivetrain.PID.kA),
                Constants.Drivetrain.Geometry.KINEMATICS,
                drivetrain::getWheelSpeeds,
                new PIDController(Constants.Drivetrain.PID.kPVel, 0, 0),
                new PIDController(Constants.Drivetrain.PID.kPVel, 0, 0),
                drivetrain::tankDriveVolts,
                drivetrain);
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
}
