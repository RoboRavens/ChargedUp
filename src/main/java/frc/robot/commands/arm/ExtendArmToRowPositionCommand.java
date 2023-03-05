package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.StateManagement.LoadTargetState;
import frc.util.StateManagement.ScoringTargetState;

public class ExtendArmToRowPositionCommand extends CommandBase {
    public ExtendArmToRowPositionCommand(ScoringTargetState scoringTargetState) {
        addRequirements(Robot.ARM_SUBSYSTEM);
    }
    
    // TODO: Implement this command
    // Remember to update arm extension state
}
