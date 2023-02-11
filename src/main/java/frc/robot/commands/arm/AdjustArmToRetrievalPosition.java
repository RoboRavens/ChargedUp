package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.scoring_states.PieceRetrievalState;

public class AdjustArmToRetrievalPosition extends CommandBase {
    private PieceRetrievalState _pieceRetrievalState;
    
    public AdjustArmToRetrievalPosition(PieceRetrievalState pieceRetrievalState) {
        _pieceRetrievalState = pieceRetrievalState;
        addRequirements(Robot.ARM_SUBSYSTEM);
    }
}
