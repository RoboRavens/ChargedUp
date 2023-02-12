package frc.util;

import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.robot.Robot;
import frc.robot.commands.arm.AdjustArmToRetrievalPosition;
import frc.robot.commands.arm.AdjustArmToRowPosition;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.robot.commands.claw.CloseClawCommand;
import frc.robot.commands.claw.OpenClawCommand;
import frc.robot.commands.command_groups.ScoreGamePieceCommand;
import frc.robot.commands.drivetrain.AlignRobotByScoringNode;
import frc.util.scoring_states.GamePieceState;
import frc.util.scoring_states.PieceRetrievalState;
import frc.util.scoring_states.RowSelectionState;

public class StateManagement {
    private boolean _alignSquareWithField = false;
    
    public void manageStates() {
        if (Robot.CLAW_SUBSYSTEM.detectsGamePiece()) {
            if (Robot.CLAW_SUBSYSTEM.isOpen()) {
                new CloseClawCommand().schedule();
            }
            // If the claw is closed (this means we have obtained a piece)
            else {
                if (Robot.LIMELIGHT_SUBSYSTEM.isInAllianceCommunity()) {
                    new AdjustArmToRowPosition(Robot.rowSelectionState).andThen(new ExtendArmCommand()).schedule();
                    Robot.LIMELIGHT_SUBSYSTEM.switchToScoringPipeline();
                    // The DrivetrainDefaultCommand recieves this value as a signal to adjust robot rotation
                    _alignSquareWithField = true;
                    if (Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER) && Robot.DRIVE_TRAIN_SUBSYSTEM.isRobotSquareWithField() && Robot.driverControlOverride == false) {
                        if (Robot.LIMELIGHT_SUBSYSTEM.isAlignedWithScoringNode()) {
                            if (Robot.OP_PAD.getButtonValue(ButtonCode.B)) {
                                // Opens the claw, waits 0.25 seconds, then simultaneously enables driver control, closes the claw, and retracts the arm
                                new ScoreGamePieceCommand().schedule();
                                clearStates();
                            }
                            else {
                                // TODO: implement LED flashing
                            }
                        }
                        else {
                            // Limelight locks onto one scoring node and robot position adjusts to align with it
                            new AlignRobotByScoringNode().schedule(); 
                        }
                    }
                    // If the robot has not finished aligning square with the field when the driver intends to score, finish aligning the robot first
                    else if (Robot.DRIVE_TRAIN_SUBSYSTEM.isRobotSquareWithField() == false) {
                        _alignSquareWithField = true;
                    }
                }
                // If the robot has a game piece but is not in the alliance community, retract the arm
                else {
                    new RetractArmCommand().schedule();
                }
            }
        }
        // If the robot does not detect a game piece
        else {
            // If the robot is either (1) in the loading zone and intends to load a piece or (2) not in any of the opponent's zones and intends to load a piece from the floor
            // TODO: Make this set of conditions more readable
            if ((Robot.LIMELIGHT_SUBSYSTEM.isInAllianceLoadingZone() && Robot.pieceRetrievalState == PieceRetrievalState.SUBSTATION)
                || (Robot.LIMELIGHT_SUBSYSTEM.isInOpponentCommunity() == false && Robot.LIMELIGHT_SUBSYSTEM.isInOpponentLoadingZone() == false && Robot.pieceRetrievalState == PieceRetrievalState.FLOOR)) {
                // Adjust the arm to the selected intake position and open the claw
                new AdjustArmToRetrievalPosition(Robot.pieceRetrievalState).andThen(new ExtendArmCommand()).schedule();
                new OpenClawCommand().schedule();
            }
            else {
                new RetractArmCommand().schedule();
            }
        }
    }

    private void clearStates() {
        Robot.pieceRetrievalState = PieceRetrievalState.CLEAR;
        Robot.gamePieceState = GamePieceState.CLEAR;
        Robot.rowSelectionState = RowSelectionState.CLEAR;
    }

    public boolean getAlignSquareWithField() {
        return _alignSquareWithField;
    }
}
