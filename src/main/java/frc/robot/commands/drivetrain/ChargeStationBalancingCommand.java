package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChargeStationBalancingCommand extends CommandBase {
    // The pid values should result in the robot applying more voltage when the pitch is higher, and vice versa
    PIDController pid = new PIDController(0.08, 0.36, 1.5);
    private double yVelocity = 0; // in meters per second

    public ChargeStationBalancingCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // Calculates the drivetrain speed based on the roll of the robot
        double currentRoll = Robot.DRIVE_TRAIN_SUBSYSTEM.getRoll();
        double pidYVelocityCalculation = pid.calculate(currentRoll, 0);
        // Ensures that the above pid calculation does not exceed the maximum drivetrain velocity
        // This also eliminates any large outliers that occur frequently with this PID configuration
        if (Math.abs(pidYVelocityCalculation) <= DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND) {
            yVelocity = pidYVelocityCalculation;
        } 
        // If the charge station is within 2 degrees of zero, the robot can stop
        // Need to double check with the game rules that this is the correct margin
        if (currentRoll < 2 && currentRoll > -2) {
            yVelocity = 0;
        }
        Robot.DRIVE_TRAIN_SUBSYSTEM.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0, // x translation
                yVelocity, // y translation
                0, // rotation
                Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation() // The angle of the robot as measured by a gyroscope.
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}
