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
import frc.util.StateManagement.DrivetrainState;
import frc.util.StateManagement.OverallState;
import edu.wpi.first.math.MathUtil;

public class DrivetrainDefaultCommand extends CommandBase {
    private boolean _followLimelight = false;
    private boolean _autoSteer = true;
    private PIDController _scoringRotationAlignPID = new PIDController(0.3, 0, 0);
    private PIDController _autoSteerPID = new PIDController(.035, 0, 0);

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

        // Set the drivetrain states and the x, y, and r values based on the overall robot state
        if (Robot.overallState == OverallState.PREPARING_TO_SCORE) {
            Robot.drivetrainState = DrivetrainState.FREEHAND_WITH_ROTATION_LOCK;
            r = getAngularVelocityForScoringAlign();
            SmartDashboard.putNumber("angular velocity pid", r);
        }
        else if (Robot.overallState == OverallState.FINAL_SCORING_ALIGNMENT) {
            Robot.drivetrainState = DrivetrainState.FINAL_SCORING_ROTATION_LOCK_AND_AUTO_ALIGN;
            r = getAngularVelocityForScoringAlign();
            // TODO: set x to align with a scoring node based on limelight input
            x = 0;
        }
        else if (Robot.overallState == OverallState.DOUBLE_SUBSTATION_PICKUP) {
            Robot.drivetrainState = DrivetrainState.DOUBLE_SUBSTATION_ALIGN;
            // TODO: set the r and x value to align with a piece on the HPS
            x = 0;
            r = 0;
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

    private double getAngularVelocityForScoringAlign() {
        // Assumes that the robot's initial rotation (0) is aligned with the scoring nodes
        // double currentRotationOffset = MathUtil.angleModulus(Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getRadians());
        // double pidCalculation = _scoringRotationAlignPID.calculate(Math.abs(currentRotationOffset));
        // double angularVelocity = pidCalculation * (currentRotationOffset < 0 ? -1 : 1);
        // if (angularVelocity > DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) {
        //     return DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        // }
        // This will need to be tested
        double angularVelocity = _scoringRotationAlignPID.calculate(Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getRadians()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        double velocityDirection = angularVelocity < 0 ? -1 : 1;
        if (Math.abs(angularVelocity) > DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) {
            return DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * velocityDirection;
        }
        else if (Math.abs(angularVelocity) < 0.5 && Math.abs(Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getRadians()) > 0.02) {
            return 0.5 * velocityDirection;
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