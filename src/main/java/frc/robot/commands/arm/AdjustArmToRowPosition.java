package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.scoring_states.RowSelectionState;

public class AdjustArmToRowPosition extends CommandBase {
    private RowSelectionState _rowSelectionState;

    public AdjustArmToRowPosition(RowSelectionState rowSelectionState) {
        _rowSelectionState = rowSelectionState;
        addRequirements(Robot.ARM_SUBSYSTEM);
    }

    // TODO: Implement this command
    // Should handle all four states: LOW, MID, HIGH, and CLEAR
    // The arm should adjust its position above the selected state, and should retract if the CLEAR state is selected
    // This should also update the row selection state (e.g. from CLEAR to MID)
}
