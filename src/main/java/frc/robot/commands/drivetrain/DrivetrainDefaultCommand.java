package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.AxisCode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.Deadband;

public class DrivetrainDefaultCommand extends CommandBase {
    private boolean _followLimelight = false;
    private boolean _autoSteer = true;
    private PIDController _followLimelightPID = new PIDController(.13, 0, 0);
    private PIDController _autoSteerPID = new PIDController(.035, 0, 0);

    public DrivetrainDefaultCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double x = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKY) * -1; // Robot.JOYSTICK.getRawAxis(1); // Positive x is away from your alliance wall.
        double y = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKX) * -1; // Robot.JOYSTICK.getRawAxis(0); // Positive y is to your left when standing behind the alliance wall.
        double r; // The angular rate of the robot.
        Rotation2d a = Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.
        // System.out.println("x before deadband: " + x);
        // System.out.println("y before deadband: " + y);
        x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);

        double rightJoystickInput = Robot.GAMEPAD.getAxis(AxisCode.RIGHTSTICKX) * -1; // Robot.JOYSTICK.getRawAxis(2);
        // System.out.println("rightJoystickInput before deadband: " + rightJoystickInput);
        rightJoystickInput = Deadband.adjustValueToZero(rightJoystickInput, Constants.JOYSTICK_DEADBAND);
        // System.out.println("rightJoystickInput after deadband: " + rightJoystickInput);

        // SmartDashboard.putNumber("Drive Time", Timer.getFPGATimestamp());
        // SmartDashboard.putNumber("Drive X", x);
        // SmartDashboard.putNumber("Drive Y", y);

        x = x * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        y = y * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;

        SmartDashboard.putBoolean("autosteer", _autoSteer);
        // var limelightAngle = this.getLimelightTargetOffset();
        // if (limelightAngle != null) {
        //     r = _followLimelightPID.calculate(limelightAngle.doubleValue());
        //     // SmartDashboard.putNumber("", r);
        // } 
        if (Math.abs(rightJoystickInput) > 0.0){
            // _followLimelightPID.reset();
            r = rightJoystickInput * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;
        // } else if (_autoSteer && Robot.DRIVE_TRAIN_SUBSYSTEM.powerIsCut() == false && (x != 0 || y != 0)) {
        //    var angularDiff = this.getDegreesToMovementDirection(x, y, a.getDegrees());
        //    double autoSteerRotationalVelocity = _autoSteerPID.calculate(angularDiff);
        //    r = autoSteerRotationalVelocity;
        } else {
            r = 0.0;
        }

        // SmartDashboard.putNumber("Drive R", r);

        if (x == 0 && y == 0 && r == 0) {
            Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            Robot.DRIVE_TRAIN_SUBSYSTEM.holdPosition();
            // System.out.println("All zeroes");
        } else {
            SmartDashboard.putNumber("x", x);
            SmartDashboard.putNumber("y", y);
            SmartDashboard.putNumber("r (rotation)", r);
            SmartDashboard.putNumber("a (angle of the robot, degrees)", a.getDegrees());
            Robot.DRIVE_TRAIN_SUBSYSTEM.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    x, // x translation
                    y, // y translation
                    r, // rotation
                    a // The angle of the robot as measured by a gyroscope.
                )
            );
            // System.out.println("x: " + x);
            // System.out.println("y: " + y);
            // System.out.println("r: " + r);
            // System.out.println("a: " + a);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public void followLimelight() {
        _followLimelight = true;
    }

    public void stopFollowingLimelight() {
        _followLimelight = false;
    }

    public void enableAutoSteer() {
        _autoSteer = true;
    }

    public void disableAutoSteer() {
        _autoSteer = false;
    }

    // private Double getLimelightTargetOffset() {
    //     if (_followLimelight == false) {
    //         return null;
    //     }

    //     if (Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted() == false) {
    //         return null;
    //     }

    //     if (Robot.LIMELIGHT_SUBSYSTEM.isAligned()) {
    //         return 0.0;
    //     }

    //     return Robot.LIMELIGHT_SUBSYSTEM.getTargetOffsetAngle();
    // }

    private double getDegreesToMovementDirection(double x, double y, double robotAngleDegrees) {
        double desiredAngleRadians = Math.atan2(y, x);
        return this.getShortestAngularDifference(robotAngleDegrees, Math.toDegrees(desiredAngleRadians));
    }

    /**
     * Gets the shortest angular difference between two points.
     * @param current current angle in degrees
     * @param target target angle in degrees
     * @return the shortest angle to get from current to target
     */
    private double getShortestAngularDifference(double current, double target) {
		current = current % 360.0;
		target = target % 360.0;
		double d = Math.abs(current - target) % 360.0; 
		double r = d > 180 ? 360 - d : d;
		
		//calculate sign 
		int sign = (current - target >= 0 && current - target <= 180) || (current - target <= -180 && current - target >= -360) ? 1 : -1; 
		r *= sign;

		return r;
    }
}