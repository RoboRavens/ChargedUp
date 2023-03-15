package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.AxisCode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.Deadband;
import frc.util.StateManagement.DrivetrainState;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.LoadTargetState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.ZoneState;
import edu.wpi.first.math.MathUtil;

public class DrivetrainDefaultCommand extends CommandBase {
    private boolean _followLimelight = false;
    private boolean _autoSteer = true;
    private PIDController _scoringRotationAlignPID = new PIDController(0.3, 0, 0);
    private PIDController _autoSteerPID = new PIDController(.035, 0, 0);
    private PIDController _yPID = new PIDController(1, 0, 0);
    private PIDController _xPID = new PIDController(1, 0, 0);
    private double _targetRotation = 0;
    // Pose2d _targetPose = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23), Rotation2d.fromDegrees(-180));
    Pose2d _targetPose = new Pose2d(Units.feetToMeters(2), Units.feetToMeters(2), Rotation2d.fromDegrees(-180));

    public DrivetrainDefaultCommand() {
        SmartDashboard.putString("DriveTrainDefaultCommandState", "constructed");
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("DriveTrainDefaultCommandState", "execute");
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

        SmartDashboard.putNumber("x pos", Robot.DRIVE_TRAIN_SUBSYSTEM.getPose().getX());
        SmartDashboard.putNumber("y pos", Robot.DRIVE_TRAIN_SUBSYSTEM.getPose().getY());

        if (Robot.drivetrainState == DrivetrainState.ROBOT_ALIGN) {
            // Set the robot to score
            // TODO: update _targetPose based on the selected scoring location
            // Get rid of the above three lines after testing
            if (Robot.overallState == OverallState.PREPARING_TO_SCORE || 
            (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE && Robot.loadState == LoadState.EMPTY && Robot.loadTargetState == LoadTargetState.DOUBLE_SUBSTATION)) {
                _targetRotation = 0;
                r = getAngularVelocityForAlignment();
                x = getXVelocity();
                y = getYVelocity();
            }
            else if (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE && Robot.loadState == LoadState.EMPTY && Robot.loadTargetState == LoadTargetState.SINGLE_SUBSTATION) {
                if (DriverStation.getAlliance() == DriverStation.getAlliance().Red) {
                    _targetRotation = 1.571; // 90 degrees
                }
                else {
                    _targetRotation =  -1.571; // -90 degrees
                }
                r = getAngularVelocityForAlignment();
                x = getXVelocity();
                y = getYVelocity();
            }
        }
        // Set the drivetrain states and the x, y, and r values based on the overall robot state
        else if (Robot.overallState == OverallState.PREPARING_TO_SCORE) {
            Robot.drivetrainState = DrivetrainState.FREEHAND_WITH_ROTATION_LOCK;
            r = getAngularVelocityForAlignment();
            SmartDashboard.putNumber("angular velocity pid", r);
        }
        else if (Robot.overallState == OverallState.LOADING) {
            Robot.drivetrainState = DrivetrainState.ACTIVELY_LOADING;
            x = 0;
            y = 0;
            r = 0;
        }
        else if (Robot.overallState == OverallState.SCORING) {
            Robot.drivetrainState = DrivetrainState.SCORING;
            x = 0;
            y = 0;
            r = 0;
        }
        else {
            Robot.drivetrainState = DrivetrainState.FREEHAND;
        }

        // apply the x, y, and r values to the drivetrain
        if (x == 0 && y == 0 && r == 0) {
            Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            Robot.DRIVE_TRAIN_SUBSYSTEM.holdPosition();
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
        }
    }

    public double getYVelocity() {
        double yOffsetFromTarget = _targetPose.getY() - Robot.DRIVE_TRAIN_SUBSYSTEM.getPose().getY();
        double ySpeed = _yPID.calculate(yOffsetFromTarget) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * -1;
        double velocityDirection = ySpeed < 0 ? -1 : 1;
        if (Math.abs(ySpeed) > DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2) {
            ySpeed = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2 * velocityDirection;
        } 
        // If the y velocity is less than 0.2 and the robot is not yet within 0.5 inches from the target y location (exact value should be updated)
        else if (Math.abs(ySpeed) < 0.2 && Math.abs(yOffsetFromTarget) > 0.0127) {
            ySpeed = 0.2 * velocityDirection;
        }
        // If the offset is within 0.5 inches, set the speed to 0 (exact value should be updated)
        else if (Math.abs(yOffsetFromTarget) < 0.0127) {
            ySpeed = 0;
        }
        return ySpeed;
    }

    public double getXVelocity() {
        double xOffsetFromTarget = _targetPose.getX() - Robot.DRIVE_TRAIN_SUBSYSTEM.getPose().getX();
        double xSpeed = _xPID.calculate(xOffsetFromTarget) * Robot.DRIVE_TRAIN_SUBSYSTEM.MAX_VELOCITY_METERS_PER_SECOND * -1;
        double velocityDirection = xSpeed < 0 ? -1 : 1;
        if (Math.abs(xSpeed) > DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2) {
            xSpeed = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2 * velocityDirection;
        }
        else if (Math.abs(xSpeed) < 0.2 && Math.abs(xOffsetFromTarget) > 0.0127) {
            xSpeed = 0.2 * velocityDirection;
        }
        // If the offset is within 0.5 inches, set the speed to 0 (exact value should be updated)
        else if (Math.abs(xOffsetFromTarget) < 0.0127) {
            xSpeed = 0;
        }
        return xSpeed;
    }

    private double getAngularVelocityForAlignment() {
        // Assumes that the robot's initial rotation (0) is aligned with the scoring nodes
        double currentRotation = Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getRadians();
        double rotationOffset = currentRotation - _targetRotation;
        SmartDashboard.putNumber("Rotation Offset", rotationOffset);
        double angularVelocity = _scoringRotationAlignPID
        .calculate(rotationOffset) 
        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        double velocityDirection = angularVelocity < 0 ? -1 : 1;
        boolean isWithinTwoHundredthsRadianOfTargetRotation = currentRotation > _targetRotation - 0.02 && currentRotation < _targetRotation + 0.02;
        SmartDashboard.putBoolean("isWithinTwoHundredthsRadianOfTargetRotation", isWithinTwoHundredthsRadianOfTargetRotation);
        // If the angular velocity is greater than the max angular velocity, set it to the max angular velocity
        if (Math.abs(angularVelocity) > DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) {
            return DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * velocityDirection;
        }
        // If the angular velocity is less than 0.4 and the robot is not within 0.02 radians of 0 degrees, set the velocity to 0.4
        else if (Math.abs(angularVelocity) < 0.4 && isWithinTwoHundredthsRadianOfTargetRotation == false) {
            return 0.4 * velocityDirection;
        }
        return angularVelocity; // angular velocity
    }

    @Override
    public void end(boolean interrupted) {
        Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        SmartDashboard.putString("end method", "end");
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