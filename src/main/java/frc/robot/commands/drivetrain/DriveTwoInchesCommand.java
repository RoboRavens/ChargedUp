package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.StateManagement.OverallState;

public class DriveTwoInchesCommand extends CommandBase {
    private Pose2d _targetPose;
    private Timer _timer = new Timer();
    private final double _twoInchesInMeters = 0.0508; // correct value is 0.0508
    private PIDController _drivePID = new PIDController(1, 0, 0);
    double xOffsetFromTarget;
    double yOffsetFromTarget;
    double xSpeed = 0;
    double ySpeed = 0;
    char _direction;

    /**
     * @param direction F for forward, B for backward, L for left, and R for right
     */
    public DriveTwoInchesCommand(char direction) {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
        _direction = direction;
        _timer.start();
    }

    @Override
    public void initialize() {
        _timer.restart();
        double currentXPosition = Robot.DRIVE_TRAIN_SUBSYSTEM.getPoseX();
        double currentYPostition = Robot.DRIVE_TRAIN_SUBSYSTEM.getPoseY();
        Rotation2d currentRotation = Robot.DRIVE_TRAIN_SUBSYSTEM.getPoseRotation();
        switch (_direction) {
            case 'F':
                _targetPose = new Pose2d(currentXPosition + _twoInchesInMeters, currentYPostition, currentRotation);
                break;
            case 'B':
                _targetPose = new Pose2d(currentXPosition - _twoInchesInMeters, currentYPostition, currentRotation);
                break;
            case 'L':
                _targetPose = new Pose2d(currentXPosition, currentYPostition + _twoInchesInMeters, currentRotation);
                break;
            case 'R':
                _targetPose = new Pose2d(currentXPosition, currentYPostition - _twoInchesInMeters, currentRotation);
                break;
            default:
                throw new IllegalArgumentException("Invalid direction");
        }
        xOffsetFromTarget = _targetPose.getX() - currentXPosition;
        yOffsetFromTarget = _targetPose.getY() - currentYPostition;
        // SmartDashboard.putNumber("Original xOffset", _targetPose.getX() - currentXPosition);
        // SmartDashboard.putNumber("Original yOffset", _targetPose.getY() - currentYPostition);
    }

    @Override
    public void execute() {
        // SmartDashboard.putBoolean("execute 2 in", true);
        if (xOffsetFromTarget != 0) {
            xOffsetFromTarget = _targetPose.getX() - Robot.DRIVE_TRAIN_SUBSYSTEM.getPose().getX();
            xSpeed = _drivePID.calculate(xOffsetFromTarget) * Robot.DRIVE_TRAIN_SUBSYSTEM.MAX_VELOCITY_METERS_PER_SECOND * -1;
            if (Math.abs(xSpeed) > DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2) {
                boolean isNegative = xSpeed < 0;
                xSpeed = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2;
                if (isNegative) xSpeed *= -1; 
            }
            else if (Math.abs(xSpeed) < 0.2) {
                boolean isNegative = xSpeed < 0;
                xSpeed = 0.2;
                if (isNegative) xSpeed *= -1;
            }
        }
        if (yOffsetFromTarget != 0) {
            yOffsetFromTarget = _targetPose.getY() - Robot.DRIVE_TRAIN_SUBSYSTEM.getPose().getY();
            ySpeed = _drivePID.calculate(yOffsetFromTarget) * Robot.DRIVE_TRAIN_SUBSYSTEM.MAX_VELOCITY_METERS_PER_SECOND * -1;
            if (Math.abs(ySpeed) > DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2) {
                boolean isNegative = ySpeed < 0;
                xSpeed = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2;
                if (isNegative) xSpeed *= -1;
            }
            else if (Math.abs(ySpeed) < 0.2) {
                boolean isNegative = ySpeed < 0;
                ySpeed = 0.2;
                if (isNegative) ySpeed *= -1;
            }
        }
        if (Robot.DRIVE_TRAIN_SUBSYSTEM.powerIsCut()) {
            if (Robot.overallState == OverallState.ENDGAME) {
                xSpeed *= 4;
                ySpeed *= 4;
            }
            else {
                xSpeed *= 2;
                ySpeed *= 2;
            }
        }
        /*
        SmartDashboard.putNumber("xOffset", xOffsetFromTarget);
        SmartDashboard.putNumber("yOffset", yOffsetFromTarget);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        */
        Robot.DRIVE_TRAIN_SUBSYSTEM.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, // x translation
                ySpeed, // y translation
                0, // rotation
                Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation() // The angle of the robot as measured by a gyroscope.
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Note: may need to reschedule the default drive command
    }

    @Override
    public boolean isFinished() {
        if (_timer.get() > 1 || (Math.abs(xOffsetFromTarget) < 0.00254 && Math.abs(yOffsetFromTarget) < 0.00254)) { // 1/10 of an inch
            // SmartDashboard.putNumber("Time taken", _timer.get());
            return true;
        }
        return false;
    }

    // Remember to add a timeout in case something goes wrong
}
