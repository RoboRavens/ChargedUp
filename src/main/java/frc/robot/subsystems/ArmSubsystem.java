package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.scoring_states.PieceRetrievalState;
import frc.util.scoring_states.RowSelectionState;

public class ArmSubsystem extends SubsystemBase {
    // Should handle all four states: LOW, MID, HIGH, and CLEAR
    // The arm should adjust its position above the selected state, and should retract if the CLEAR state is selected
    // This should also update the row selection state (e.g. from CLEAR to MID)
    public void adjustArmToRowPosition(RowSelectionState rowSelectionState) {}

    // Should handle all three states: FLOOR, SUBSTATION, and CLEAR
    // The arm should adjust its position to retrieve game pieces from the selected state, and should retract if the CLEAR state is selected
    // This should also update the piece retrieval state (e.g. from SUBSTATION to FLOOR)
    public void adjustArmToRetrievalPosition(PieceRetrievalState pieceRetrievalState) {}
}
