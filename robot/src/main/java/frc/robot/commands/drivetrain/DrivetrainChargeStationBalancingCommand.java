package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainChargeStationBalancingCommand extends CommandBase {
    private double xVelocity = 0; // in meters per second

    public DrivetrainChargeStationBalancingCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    @Override
    public void initialize() {
        // SmartDashboard.putBoolean("balancing init", true);
        // SmartDashboard.putBoolean("balancing end", false);
    }

    @Override
    public void execute() {
        // Calculates the drivetrain speed based on the roll of the robot
        double currentRoll = Robot.DRIVE_TRAIN_SUBSYSTEM.getRoll();
        double xVelocityCalculation = currentRoll / 50 * -1;
        // Ensures that the above pid calculation does not exceed the maximum drivetrain velocity
        // This also eliminates any large outliers that occur frequently with this PID configuration
        if (Math.abs(xVelocityCalculation) <= DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND) {
            xVelocity = xVelocityCalculation;
        } 
        // "A CHARGE STATION is considered LEVEL if it is within approximately 2½° of parallel to FIELD carpet"
        if (currentRoll < 2.5 && currentRoll > -2.5) {
            xVelocity = 0;
        }
        SmartDashboard.putNumber("xVelocity", xVelocity);
        // SmartDashboard.putNumber("Roll (ChargeStationBalancingCommand)", currentRoll);
        // SmartDashboard.putNumber("Velocity (ChargeStationBalancingCommand)", xVelocity);
        Robot.DRIVE_TRAIN_SUBSYSTEM.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, // x translation
                0, // y translation
                0, // rotation
                Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation() // The angle of the robot as measured by a gyroscope.
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0, 0, 0));
        // SmartDashboard.putBoolean("balancing end", true);
        // SmartDashboard.putBoolean("balancing init", false);
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()) {
            return false;
        }
        else {
            if (Math.abs(Robot.DRIVE_TRAIN_SUBSYSTEM.getRoll()) < 2.5) {
                return true;
            }
            return false;
        }
    }
}
