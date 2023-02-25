package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ExtendArmCommand extends CommandBase {
    public ExtendArmCommand() {
        addRequirements(Robot.ARM_SUBSYSTEM);
    }
    
    // TODO: Implement this command
    // Remember to check if the robot is any of the opposite alliance's zones
    // If so, the arm should not extend
    // Maybe indicate to the driver somehow that they are in the wrong zone
}
