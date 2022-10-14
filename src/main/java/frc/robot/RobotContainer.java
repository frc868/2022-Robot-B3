// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.techhounds.houndutil.houndlib.auto.AutoManager;
import com.techhounds.houndutil.houndlib.auto.AutoPath;
import com.techhounds.houndutil.houndlib.auto.AutoRoutine;
import com.techhounds.houndutil.houndlib.auto.trajectoryloader.TrajectoryLoader;
import com.techhounds.houndutil.houndlib.auto.trajectoryloader.TrajectorySettings;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OI;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterSetSpeed;
import frc.robot.commands.TurnToBall;
import frc.robot.commands.TurnToGoal;
import frc.robot.commands.auton.DrivetrainTrajectoryCommand;
import frc.robot.commands.auton.ShootSequence;
import frc.robot.commands.auton.paths.trajectory.FiveBall;
import frc.robot.commands.auton.paths.trajectory.FourBall;
import frc.robot.commands.auton.paths.trajectory.ThreeBall1;
import frc.robot.commands.auton.paths.trajectory.TwoBall;
import frc.robot.sensors.Astra;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Misc;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Hopper hopper = new Hopper();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();
    private final Shooter shooter = new Shooter();
    private final Limelight limelight = new Limelight();
    private final Astra astra = new Astra();
    @SuppressWarnings("unused")
    private final Misc misc = new Misc();

    SlewRateLimiter xLimiter = new SlewRateLimiter(10);
    SlewRateLimiter yLimiter = new SlewRateLimiter(10);

    XboxController driverController;
    XboxController operatorController;
    SendableChooser<AutoRoutine> chooser = new SendableChooser<AutoRoutine>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        if (RobotBase.isReal()) { // prevents annoying joystick disconnected warning
            configureControllerBindings();
        }
        configureAuton();

        LoggingManager.getInstance().addGroup("Commands", new LogGroup(
                new Logger[] {
                        new SendableLogger("Intake Up", new InstantCommand(intake::setUp, intake)),
                        new SendableLogger("Intake Down", new InstantCommand(intake::setDown, intake)),
                        new SendableLogger("Extend Climber Locks",
                                new InstantCommand(climber::extendLock, climber)),
                        new SendableLogger("Retract Climber Locks",
                                new InstantCommand(climber::retractLock, climber)),
                        new SendableLogger("Extend Climber Stage 2",
                                new InstantCommand(climber::extendSecondStage, climber)),
                        new SendableLogger("Retract Climber Stage 2",
                                new InstantCommand(climber::retractSecondStage, climber)),
                        new SendableLogger("Run Hopper",
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper)),
                        new SendableLogger("Run Intake",
                                new StartEndCommand(intake::runMotor, intake::stopMotor, intake)),
                        new SendableLogger("Gatekeepers In",
                                new InstantCommand(hopper::gatekeepersIn, hopper)),
                        new SendableLogger("Gatekeepers Out",
                                new InstantCommand(hopper::gatekeepersOut, hopper)),
                        new SendableLogger("Run Shooter", new RunShooter(shooter, limelight)),
                        new SendableLogger("Run Shooter Locked Speed", new RunShooter(shooter, limelight)),
                        new SendableLogger("Turn To Goal", new TurnToGoal(drivetrain, limelight)),
                        new SendableLogger("Turn To Ball", new TurnToBall(drivetrain, astra)),
                        new SendableLogger("Test", new PrintCommand("hi")),
                        new SendableLogger("Limelight Off", new InstantCommand(limelight::setDriverAssistMode) {
                            @Override
                            public boolean runsWhenDisabled() {
                                return true;
                            }
                        }),
                        new SendableLogger("Limelight On", new InstantCommand(limelight::setShootingMode) {
                            @Override
                            public boolean runsWhenDisabled() {
                                return true;
                            }
                        }),
                }));
    }

    private void configureControllerBindings() {
        driverController = new XboxController(OI.DRIVER_PORT);
        operatorController = new XboxController(OI.OPERATOR_PORT);

        drivetrain.setDefaultCommand(
                new DefaultDrive(drivetrain, driverController::getLeftY, driverController::getRightY));
        climber.setDefaultCommand(
                new RunCommand(() -> {
                    climber.setSpeedLeft(operatorController.getLeftY());
                    climber.setSpeedRight(operatorController.getRightY());
                }, climber));

        // Driver button A, intake down
        new JoystickButton(driverController, Button.kA.value)
                .whenPressed(new InstantCommand(intake::setDown, intake));
        // Driver button Y, intake up
        new JoystickButton(driverController, Button.kY.value)
                .whenPressed(new InstantCommand(intake::setUp, intake));

        // Driver button B, climber locks extend
        new JoystickButton(driverController, Button.kB.value)
                .whenPressed(new InstantCommand(climber::extendLock));
        // Driver button X, climber locks retract
        new JoystickButton(driverController, Button.kX.value)
                .whenPressed(new InstantCommand(climber::retractLock));

        // Driver button Start, engage climber stage 2
        new JoystickButton(driverController, Button.kStart.value)
                .whenPressed(new InstantCommand(climber::extendSecondStage));
        // Driver button Back, disengage climber stage 2
        new JoystickButton(driverController, Button.kBack.value)
                .whenPressed(new InstantCommand(climber::retractSecondStage));

        // Driver button RB, run hopper and intake while held
        new JoystickButton(driverController, Button.kRightBumper.value)
                .whenHeld(
                        new ParallelCommandGroup(
                                new StartEndCommand(hopper::runMotor, hopper::stopMotor, hopper),
                                new StartEndCommand(intake::runMotor, intake::stopMotor, intake)));
        // Driver button LB, run hopper and intake in reverse while held
        new JoystickButton(driverController, Button.kLeftBumper.value)
                .whenHeld(
                        new ParallelCommandGroup(
                                new StartEndCommand(hopper::reverseMotor, hopper::stopMotor, hopper),
                                new StartEndCommand(intake::reverseMotor, intake::stopMotor, intake)));

        // Operator button LB, toggle shooter run with limelight regression
        new JoystickButton(operatorController, Button.kLeftBumper.value)
                .toggleWhenPressed(new RunShooterSetSpeed(2000.0 / 60.0, shooter));

        // Operator button X, gatekeepers out
        new JoystickButton(operatorController, Button.kX.value)
                .whenPressed(new InstantCommand(hopper::gatekeepersOut, hopper));
        // Operator button B, gatekeepers in
        new JoystickButton(operatorController, Button.kB.value)
                .whenPressed(new InstantCommand(hopper::gatekeepersIn, hopper));

        new JoystickButton(operatorController, Button.kA.value).whenPressed(
                new ShootSequence(drivetrain, shooter, limelight, hopper));

        // // Operator D-Pad North, turn to goal
        new POVButton(operatorController, 0)
                .whenPressed(new TurnToGoal(drivetrain, limelight));
        // Operator D-Pad South, turn to ball
        new POVButton(operatorController, 180)
                .whenPressed(new TurnToBall(drivetrain, astra));
    }

    private void configureAuton() {
        TrajectoryLoader.addSettings(new TrajectorySettings("2Ball1").withMaxVelocity(2));
        TrajectoryLoader.loadAutoPaths();

        AutoManager.getInstance().addRoutine(new AutoRoutine("Two Ball: Left",
                new TwoBall(TrajectoryLoader.getAutoPaths("2Ball1"), drivetrain, shooter, intake, hopper, limelight)));

        AutoManager.getInstance().addRoutine(new AutoRoutine("Two Ball: Middle",
                new TwoBall(TrajectoryLoader.getAutoPaths("2Ball2"), drivetrain, shooter, intake, hopper, limelight)));

        AutoManager.getInstance().addRoutine(new AutoRoutine("Two Ball: Right",
                new TwoBall(TrajectoryLoader.getAutoPaths("2Ball3"), drivetrain, shooter, intake, hopper, limelight)));

        AutoManager.getInstance().addRoutine(new AutoRoutine("Three Ball: Shoot First", new ThreeBall1(
                TrajectoryLoader.getAutoPaths("3Ball1"), drivetrain, shooter, intake, hopper, limelight)));

        AutoManager.getInstance()
                .addRoutine(new AutoRoutine("Four Ball: Hangar and HP Station",
                        new FourBall(TrajectoryLoader.getAutoPaths("4Ball.To2", "4Ball.To3and4", "4Ball.ToGoal"),
                                drivetrain, shooter, intake, hopper, limelight)));

        AutoManager.getInstance()
                .addRoutine(new AutoRoutine("Five Ball",
                        new FiveBall(TrajectoryLoader.getAutoPaths("5Ball.To2and3", "5Ball.To4and5", "5Ball.ToGoal"),
                                drivetrain, shooter, intake, hopper, limelight, astra)));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Test Trajectory",
                        new DrivetrainTrajectoryCommand(TrajectoryLoader.getAutoPath("TestTrajectory").getTrajectory(),
                                drivetrain),
                        TrajectoryLoader.getAutoPaths("TestTrajectory")));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Java Traj",
                        new DrivetrainTrajectoryCommand(createJavaTraj(), drivetrain),
                        new ArrayList<AutoPath>(
                                List.of(new AutoPath("javaTraj", createJavaTraj())))));
    }

    public Trajectory createJavaTraj() {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.Drivetrain.PID.kS,
                        Constants.Drivetrain.PID.kV,
                        Constants.Drivetrain.PID.kA),
                Constants.Drivetrain.Geometry.KINEMATICS,
                10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.Auton.MAX_VELOCITY,
                Constants.Auton.MAX_ACCELERATION)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.Drivetrain.Geometry.KINEMATICS)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(1, -1, new Rotation2d(-Math.PI / 2.0)),
                config);

        return exampleTrajectory;
    }
}
