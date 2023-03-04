package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.StateManagement.LoadTargetState;

public class ExtendArmToRetrievalPositionCommand extends CommandBase {
    public ExtendArmToRetrievalPositionCommand(LoadTargetState loadTargetState) {
        addRequirements(Robot.ARM_SUBSYSTEM);
    }
    
    // TODO: Implement this command
    // Remember to update arm extension state
}
