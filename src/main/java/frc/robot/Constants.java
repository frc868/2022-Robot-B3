package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The container for robot-wide numerical or boolean constants. This should not
 * be used for any other purpose.
 *
 * @author dr
 */
public final class Constants {
    enum ControllerType {
        XboxController,
        FlightStick
    }

    public static final boolean DEBUG_MODE = false;

    public static final ControllerType CONTROLLER_TYPE = ControllerType.XboxController;

    public static final class Drivetrain {
        public static final class CANIDs {
            public static final int L_PRIMARY = 4;
            public static final int L_SECONDARY = 6;
            public static final int R_PRIMARY = 1;
            public static final int R_SECONDARY = 3;
        }

        public static final class PID {
            public static final double kS = 0.2068;
            public static final double kV = 2.5188;
            public static final double kA = 0.73449;
            public static final double kPVel = 3.6295;

            public static final class DriveStraight {
                public static final double kP = 1.0;
                public static final double kI = 0.0;
                public static final double kD = 0.1;
            }

            public static final class TurnToAngle {
                public static final double kP = 1;
                public static final double kI = 0;
                public static final double kD = 0;
            }

            public static final class TurnToBall {
                public static final double kP = 0.020;
                public static final double kI = 0.04;
                public static final double kD = 0.0045;
            }

            public static final class TurnToGoal {
                public static final double kP = 0.023;
                public static final double kI = 0.0090;
                public static final double kD = 0.0006;
            }
        }

        public static final double ENCODER_DISTANCE_TO_METERS = 1.0 * Math.PI
                * Constants.Drivetrain.Geometry.WHEEL_DIAMETER_METERS / 8.68; // encoder counts per revolution * (2 * pi
                                                                              // *
                                                                              // wheel radius) / gear ratio

        public static final class Geometry {
            /** Distance between centers of right and left wheels on robot. */
            public static final double TRACK_WIDTH_METERS = 0.62;
            public static final double WHEEL_DIAMETER_METERS = 0.14;
            public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
                    Constants.Drivetrain.Geometry.TRACK_WIDTH_METERS);
        }
    }

    public static final class OI {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class Hopper {
        public static final class CANIDs {
            public static final int MOTOR = 7;
        }

        public static final class Solenoids {
            public static final int GATEKEEPER_CHANNEL_1 = 1;
            public static final int GATEKEEPER_CHANNEL_2 = 6;
        }

        public static final boolean IS_INVERTED = false;
    }

    public static final class Intake {
        public static final class CANIDs {
            public static final int MOTOR = 10;
        }

        public static final class Solenoids {
            public static final int INTAKE_CHANNEL_1 = 0;
            public static final int INTAKE_CHANNEL_2 = 7;
        }
    }

    public static final class Shooter {
        public static final class CANIDs {
            public static final int PRIMARY = 8;
            public static final int SECONDARY = 13;
        }

        public static final double kP = 0.13887;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.032717;
        public static final double kV = 0.12396;
        public static final double kA = 0.010278;
    }

    public static final class Limelight {

        // Goal and angle measurements
        public static final double LL_HEIGHT = 22.0;
        public static final double HUB_HEIGHT = 101.0;
        public static final double ANGLE = 29.6375;

        // Good shot distances
        public static final double HIGH_GOAL_SHOT_DISTANCE = 6.75;
        public static final double LOW_GOAL_SHOT_DISTANCE = 0.0; // untested
    }

    public static final class Climber {
        public static final class CANIDs {
            public static final int PRIMARY = 12;
            public static final int SECONDARY = 9;
        }

        public static final class Solenoids {
            public static final int[] CLIMBER_2ND_STAGE = { 2, 5 };
            public static final int[] CLIMBER_LOCK = { 3, 4 };

        }
    }

    public static final class Teleop {
        public static final double POWER_LIMIT = 0.1;
    }

    public static final class Auton {
        public static final double MAX_VELOCITY = 1;
        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_ANGULAR_VELOCITY = Math.PI / 4;
        public static final double MAX_ANGULAR_ACCELERATION = Math.PI;

    }
}
