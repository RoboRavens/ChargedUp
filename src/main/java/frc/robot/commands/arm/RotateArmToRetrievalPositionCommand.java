package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.StateManagementNew.LoadTargetState;

public class RotateArmToRetrievalPositionCommand extends CommandBase {
    
    public RotateArmToRetrievalPositionCommand(LoadTargetState ground) {
        addRequirements(Robot.ARM_SUBSYSTEM);
    }

    // TODO: Implement this command
    // Remember to update the arm state
}
