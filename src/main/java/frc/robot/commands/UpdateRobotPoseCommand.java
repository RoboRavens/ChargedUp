package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * A command to drive the robot with joystick input (passed in as
 * {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command
 * this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class UpdateRobotPoseCommand extends CommandBase {

    public void enableRobotPose() {
        //addRequirements(Robot.LIMELIGHT_SUBSYSTEM);

    }

    @Override
    public void execute() {
        Robot.LIMELIGHT_SUBSYSTEM.getRobotPose();
    }
}