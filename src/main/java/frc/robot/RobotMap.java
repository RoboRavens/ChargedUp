package frc.robot;

public class RobotMap {
    // DRIVETRAIN
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5715; // The left-to-right distance between the drivetrain wheels
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5715; // The front-to-back distance between the drivetrain wheels.

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 24;

    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(73.301); // practice
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(130); // competition

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(320.625); // practice
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(79); // competition

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(191.338); // practice
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(240); // competition

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(201.885); // practice
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(288); // competition
}
