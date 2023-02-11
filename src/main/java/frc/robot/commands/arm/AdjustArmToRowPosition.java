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
}
