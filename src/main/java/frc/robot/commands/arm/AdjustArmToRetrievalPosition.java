package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.StateManagementNew.ArmExtensionState;
import frc.util.StateManagementNew.ArmRotationState;
import frc.util.scoring_states.PieceRetrievalState;

public class AdjustArmToRetrievalPosition extends CommandBase {
    
    public AdjustArmToRetrievalPosition(ArmRotationState armRotationState) {
        addRequirements(Robot.ARM_SUBSYSTEM);
    }

    // TODO: Implement this command
    // Should handle all three states: FLOOR, SUBSTATION, and CLEAR
    // The arm should adjust its position to retrieve game pieces from the selected state, and should retract if the CLEAR state is selected
    // This should also update the piece retrieval state (e.g. from SUBSTATION to FLOOR)
}
