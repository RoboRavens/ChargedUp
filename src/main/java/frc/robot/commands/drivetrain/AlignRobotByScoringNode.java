package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AlignRobotByScoringNode extends CommandBase {
    public AlignRobotByScoringNode() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }
}
