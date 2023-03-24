package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.AxisCode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.Deadband;
import frc.util.Scale;
import frc.util.Slew;
import frc.util.StateManagement.DrivetrainState;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.LoadTargetState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.ZoneState;
import frc.util.drive.AngularPositionHolder;
import frc.util.drive.ChassisSpeedsExtensions;

public class DrivetrainDefaultCommand extends CommandBase {
    private PIDController _scoringRotationAlignPID = new PIDController(0.3, 0, 0);
    private PIDController _yPID = new PIDController(.3, 0, 0);
    private PIDController _xPID = new PIDController(.3, 0, 0);

    // Pose2d _targetPose = new Pose2d(Units.feetToMeters(2), Units.feetToMeters(2), Rotation2d.fromDegrees(-180));

    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
    private double _velocityXSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_X_VELOCITY;
    private double _velocityYSlewRate = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_Y_VELOCITY;
    private double _angularSlewRate = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / Constants.SLEW_FRAMES_TO_MAX_ANGULAR_VELOCITY;

    public boolean CutPower = false;

    public DrivetrainDefaultCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
        _scoringRotationAlignPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        AngularPositionHolder.GetInstance().reset();
        _chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    @Override
    public void execute() {
        double controllerDirection = Robot.allianceColor == Alliance.Red ? 1 : -1;
        double x = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKY) * controllerDirection; // Positive x is away from your alliance wall.
        double y = Robot.GAMEPAD.getAxis(AxisCode.LEFTSTICKX) * controllerDirection; // Positive y is to your left when standing behind the alliance wall.
        double r = Robot.GAMEPAD.getAxis(AxisCode.RIGHTSTICKX) * -1; // The angular rate of the robot.
        Rotation2d a = Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.

        x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);
        r = Deadband.adjustValueToZero(r, Constants.JOYSTICK_DEADBAND);

        x = x * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        y = y * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;

        if (Robot.drivetrainState == DrivetrainState.ROBOT_ALIGN) {
            // Set the robot to score
            // TODO: update _targetPose based on the selected scoring location
            // Get rid of the above three lines after testing
            
            double targetAngle = r;

            if (Robot.loadState == LoadState.EMPTY && Robot.loadTargetState == LoadTargetState.DOUBLE_SUBSTATION) {
                double targetAngleDegrees = DriverStation.getAlliance() == Alliance.Red ? 180 : 0;  // Both Alliance.Blue and Alliance.Invalid are treated as blue alliance
                targetAngle = Math.toRadians(targetAngleDegrees);
            }
            else if (Robot.loadState == LoadState.EMPTY && Robot.loadTargetState == LoadTargetState.SINGLE_SUBSTATION) {
                double targetAngleDegrees = DriverStation.getAlliance() == Alliance.Red ? 90 : -90; // Both Alliance.Blue and Alliance.Invalid are treated as blue alliance
                targetAngle = Math.toRadians(targetAngleDegrees);
            }
            else if (Robot.loadState == LoadState.LOADED) {
                double targetAngleDegrees = DriverStation.getAlliance() == Alliance.Red ? 0 : 180; // Both Alliance.Blue and Alliance.Invalid are treated as blue alliance
                targetAngle = Math.toRadians(targetAngleDegrees);
            }

            if (Robot.ODOMETRY_OVERRIDE == false) {
                r = getAngularVelocityForAlignment(targetAngle);
            }

            x = getXVelocity();
            y = getYVelocity();

            /* 

            if (true || //Robot.overallState == OverallState.PREPARING_TO_SCORE || 
            (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE && Robot.loadState == LoadState.EMPTY && Robot.loadTargetState == LoadTargetState.DOUBLE_SUBSTATION)) {
                double targetAngleDegrees = DriverStation.getAlliance() == Alliance.Red ? 0 : 180;
                double targetAngle = Math.toRadians(targetAngleDegrees);
                r = getAngularVelocityForAlignment(targetAngle);
                x = getXVelocity();
                y = getYVelocity();
            }
            else if (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE && Robot.loadState == LoadState.EMPTY && Robot.loadTargetState == LoadTargetState.SINGLE_SUBSTATION) {
                double targetAngleDegrees = DriverStation.getAlliance() == Alliance.Red ? 90 : -90; // Both Alliance.Blue and Alliance.Invalid are treated as blue alliance
                double targetAngle = Math.toRadians(targetAngleDegrees);
                r = getAngularVelocityForAlignment(targetAngle);
                x = getXVelocity();
                y = getYVelocity();
            }
            */
        }
        // Set the drivetrain states and the x, y, and r values based on the overall robot state
        else if (Robot.overallState == OverallState.PREPARING_TO_SCORE) {
            Robot.drivetrainState = DrivetrainState.FREEHAND_WITH_ROTATION_LOCK;
            double targetAngleDegrees = DriverStation.getAlliance() == Alliance.Red ? 0 : 180; // Both Alliance.Blue and Alliance.Invalid are treated as blue alliance
            double targetAngle = Math.toRadians(targetAngleDegrees);
            
            if (Robot.ODOMETRY_OVERRIDE == false) {
                r = getAngularVelocityForAlignment(targetAngle); 
            }
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

        

        SmartDashboard.putNumber("angular velocity pid", r);

        r = AngularPositionHolder.GetInstance().getAngularVelocity(r, a.getRadians());

        // scale drive velocity and rotation
        double scale = 1;
        if (this.CutPower) {
            scale = Robot.overallState == OverallState.ENDGAME ? .25 : .5;
        }

        double scaleForArmExt = Scale.FromPercentage(1.0 - Robot.ARM_SUBSYSTEM.getExtensionPercentOfMaximumNativeUnits(), Constants.DRIVE_SPEED_SCALE_AT_ARM_MAX_EXTENSION, 1);
        scale = Math.min(scale, scaleForArmExt);

        x = x * scale;
        y = y * scale;
        r = r * scale;

        // give calculated x,y,r to drive command
        SmartDashboard.putNumber("DriveDefaultCmd x", x);
        SmartDashboard.putNumber("DriveDefaultCmd y", y);
        SmartDashboard.putNumber("DriveDefaultCmd r (rotation)", r);
        SmartDashboard.putNumber("DriveDefaultCmd a (gyro)", a.getDegrees());
        
        var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x, // x translation
            y, // y translation
            r, // rotation
            a // The angle of the robot as measured by a gyroscope.
        );

        targetChassisSpeeds.vxMetersPerSecond = Slew.GetSlewedTarget(_velocityXSlewRate, targetChassisSpeeds.vxMetersPerSecond, _chassisSpeeds.vxMetersPerSecond);
        targetChassisSpeeds.vyMetersPerSecond = Slew.GetSlewedTarget(_velocityYSlewRate, targetChassisSpeeds.vyMetersPerSecond, _chassisSpeeds.vyMetersPerSecond);
        targetChassisSpeeds.omegaRadiansPerSecond = Slew.GetSlewedTarget(_angularSlewRate, targetChassisSpeeds.omegaRadiansPerSecond, _chassisSpeeds.omegaRadiansPerSecond);
        _chassisSpeeds = targetChassisSpeeds;

        if (ChassisSpeedsExtensions.IsZero(targetChassisSpeeds)
          && Robot.zoneState == ZoneState.ALLIANCE_CHARGE_STATION
          && Robot.overallState == OverallState.ENDGAME) {
            Robot.DRIVE_TRAIN_SUBSYSTEM.holdPosition();
        } else {
            Robot.DRIVE_TRAIN_SUBSYSTEM.drive(targetChassisSpeeds);
        }
    }

    public double getYVelocity() {
        var target = Robot.TABLET_SCORING_SUBSYSTEM.GetScoringCoordinates();
        if (target == null) {
            SmartDashboard.putNumber("Score Target Y", -1);
            return 0;
        }

        SmartDashboard.putNumber("Score Target Y", target.getY());
        double yOffsetFromTarget = target.getY() - Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getY();
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
        var target = Robot.TABLET_SCORING_SUBSYSTEM.GetScoringCoordinates();
        if (target == null) {
            SmartDashboard.putNumber("Score Target X", -1);
            return 0;
        }

        SmartDashboard.putNumber("Score Target X", target.getX());
        double xOffsetFromTarget = target.getX() - Robot.POSE_ESTIMATOR_SUBSYSTEM.getCurrentPose().getX();
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

    private double getAngularVelocityForAlignment(double targetRotation) {
        // Assumes that the robot's initial rotation (0) is aligned with the scoring nodes
        double currentRotation = Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation().getRadians();
        double rotationOffset = currentRotation - targetRotation;
        SmartDashboard.putNumber("Rotation Offset", rotationOffset);
        double angularVelocity = _scoringRotationAlignPID
        .calculate(rotationOffset) 
        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        double velocityDirection = angularVelocity < 0 ? -1 : 1;
        boolean isWithinTwoHundredthsRadianOfTargetRotation = currentRotation > targetRotation - 0.02 && currentRotation < targetRotation + 0.02;
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