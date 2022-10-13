// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.PathPlanner;
import com.techhounds.houndutil.houndlib.auto.AutoManager;
import com.techhounds.houndutil.houndlib.auto.AutoPath;
import com.techhounds.houndutil.houndlib.auto.AutoRoutine;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.Logger;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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

    XboxController driverController = new XboxController(OI.DRIVER_PORT);
    XboxController operatorController = new XboxController(OI.OPERATOR_PORT);
    SendableChooser<AutoRoutine> chooser = new SendableChooser<AutoRoutine>();
    HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        // Configure the button bindings
        drivetrain.setDefaultCommand(
                new DefaultDrive(drivetrain, driverController::getLeftY, driverController::getRightY));
        climber.setDefaultCommand(
                new RunCommand(() -> {
                    climber.setSpeedLeft(operatorController.getLeftY());
                    climber.setSpeedRight(operatorController.getRightY());
                }, climber));
        configureButtonBindings();
        loadTrajectories();
        configureAuton();
        SmartDashboard.putData(new InstantCommand(() -> {
            System.out.println("test");
            AutoManager.getInstance().getField().getObject("Traj")
                    .setTrajectory(new Trajectory());
            System.out
                    .println(AutoManager.getInstance().getField().getObject("Traj").getPose());
        }));

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

    private void loadTrajectories() {
        for (String name : new String[] {
                "2Ball1",
                "2Ball2",
                "2Ball3",
                "3Ball1",
                "3Ball2-FullPath",
                "3Ball2.To2",
                "3Ball2.To3andGoal",
                "4Ball-FullPath",
                "4Ball.To2",
                "4Ball.To3and4",
                "4Ball.ToGoal",
                "5Ball-FullPath",
                "5Ball.To2and3",
                "5Ball.To4and5",
                "5Ball.ToGoal",
                "TestTrajectory" }) {
            try {
                Trajectory trajectory = PathPlanner.loadPath(name,
                        Constants.Auton.MAX_VELOCITY,
                        Constants.Auton.MAX_ACCELERATION);

                trajectories.put(name, trajectory);
            } catch (Exception ex) {
                ex.printStackTrace();
                DriverStation.reportError("Unable to open trajectory: " + name,
                        ex.getStackTrace());
            }
        }

        trajectories.put("javaTraj", createJavaTraj());
    }

    private void configureAuton() {
        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Two Ball: Left",
                        new TwoBall(trajectories.get("2Ball1"), drivetrain, shooter, intake, hopper, limelight),
                        new ArrayList<AutoPath>(List.of(new AutoPath("2Ball1", trajectories.get("2Ball1"))))));
        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Two Ball: Middle",
                        new TwoBall(trajectories.get("2Ball2"), drivetrain, shooter, intake, hopper, limelight),
                        new ArrayList<AutoPath>(List.of(new AutoPath("2Ball2", trajectories.get("2Ball2"))))));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Two Ball: Right",
                        new TwoBall(trajectories.get("2Ball3"), drivetrain, shooter, intake, hopper, limelight),
                        new ArrayList<AutoPath>(List.of(new AutoPath("2Ball3", trajectories.get("2Ball3"))))));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Three Ball: Shoot First",
                        new ThreeBall1(trajectories.get("3Ball1"), drivetrain, shooter, intake, hopper, limelight),
                        new ArrayList<AutoPath>(List.of(new AutoPath("3Ball1", trajectories.get("3Ball1"))))));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Four Ball: Hangar and HP Station",
                        new FourBall(new HashMap<String, Trajectory>(
                                Map.ofEntries(Map.entry("4Ball.To2", trajectories.get("4Ball.To2")),
                                        Map.entry("4Ball.To3and4", trajectories.get("4Ball.To3and4")),
                                        Map.entry("4Ball.ToGoal", trajectories.get("4Ball.ToGoal")))),
                                drivetrain, shooter, intake, hopper, limelight),
                        new ArrayList<AutoPath>(List.of(
                                new AutoPath("4Ball.To2", trajectories.get("4Ball.To2")),
                                new AutoPath("4Ball.To3and4", trajectories.get("4Ball.To3and4")),
                                new AutoPath("4Ball.ToGoal", trajectories.get("4Ball.ToGoal"))))));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Five Ball",
                        new FiveBall(
                                new HashMap<String, Trajectory>(
                                        Map.ofEntries(Map.entry("5Ball.To2and3", trajectories.get("5Ball.To2and3")),
                                                Map.entry("5Ball.To4and5", trajectories.get("5Ball.To4and5")),
                                                Map.entry("5Ball.ToGoal", trajectories.get("5Ball.ToGoal")))),
                                drivetrain, shooter, intake, hopper, limelight, astra),
                        new ArrayList<AutoPath>(
                                List.of(
                                        new AutoPath("5Ball.To2and3", trajectories.get("5Ball.To2and3")),
                                        new AutoPath("5Ball.To4and5", trajectories.get("5Ball.To4and5")),
                                        new AutoPath("5Ball.ToGoal", trajectories.get("5Ball.ToGoal"))))));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Test Trajectory",
                        new DrivetrainTrajectoryCommand(trajectories.get("TestTrajectory"), drivetrain),
                        new ArrayList<AutoPath>(List.of(new AutoPath("Traj", trajectories.get("TestTrajectory"))))));

        AutoManager.getInstance().addRoutine(
                new AutoRoutine(
                        "Java Traj",
                        new DrivetrainTrajectoryCommand(trajectories.get("javaTraj"), drivetrain),
                        new ArrayList<AutoPath>(
                                List.of(new AutoPath("javaTraj", trajectories.get("javaTraj"))))));

    }

    private void configureButtonBindings() {
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

    // public Command getAutonomousCommand() {
    // Trajectory trajectory = trajectories.get("5Ball");
    // Field2d field = new Field2d();
    // field.getObject("traj1").setTrajectory(trajectories.get("5Ball.To2and3"));
    // field.getObject("traj2").setTrajectory(trajectories.get("5Ball.To4and5"));
    // field.getObject("traj3").setTrajectory(trajectories.get("5Ball.ToGoal"));
    // SmartDashboard.putData(field);
    // // drivetrain.resetOdometry(trajectory.getInitialPose());
    // return new DrivetrainTrajectoryCommand(trajectory, drivetrain);
    // }
}
