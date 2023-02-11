package frc.util;

import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.robot.Robot;
import frc.robot.commands.arm.AdjustArmToRetrievalPosition;
import frc.robot.commands.arm.AdjustArmToRowPosition;
import frc.robot.commands.claw.CloseClawCommand;
import frc.robot.commands.claw.OpenClawCommand;
import frc.robot.commands.drivetrain.AlignRobotByScoringNode;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.util.scoring_states.GamePieceState;
import frc.util.scoring_states.PieceRetrievalState;
import frc.util.scoring_states.RowSelectionState;

public class StateManagement {
    public void manageStates() {
        if (Robot.CLAW_SUBSYSTEM.detectsGamePiece()) {
            if (Robot.CLAW_SUBSYSTEM.isOpen()) {
                new CloseClawCommand().schedule();
            }
            else if (Robot.CLAW_SUBSYSTEM.isOpen() == false) {
                if (Robot.LIMELIGHT_SUBSYSTEM.isInAllianceCommunity()) {
                    new AdjustArmToRowPosition(Robot.rowSelectionState).schedule();
                    // TODO: open the limelight pipeline (to detect scoring locations)
                    // TODO: align robot drivetrain square with the field (ideally this can occur while the robot is driving to score)
                    if (Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER) && Robot.DRIVE_TRAIN_SUBSYSTEM.isRobotSquareWithField()) {
                        // Limelight locks onto one scoring node and robot position adjusts to align with it
                        new AlignRobotByScoringNode().schedule(); 
                        if (Robot.LIMELIGHT_SUBSYSTEM.isAlignedWithScoringNode()) {
                            // TODO: implement LED flashing
                            if (Robot.OP_PAD.getButtonValue(ButtonCode.B)) {
                                new OpenClawCommand().schedule();
                                // TODO: reschedule default drivetrain command (this enables driver control again)
                                Robot.DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(new DrivetrainDefaultCommand());
                                // TODO: wait some time, then close the claw again
                                clearStates();
                            }
                        }
                    }
                    // Change to "if the robot is not square with the field"
                    else if (true) {
                        // Square the robot with the field
                    }
                }
            }
        }
        else {
            if (Robot.LIMELIGHT_SUBSYSTEM.isInAllianceLoadingZone()) {
                // If the claw does not have a game piece and it is in our alliance's loading zone ->
                // Adjust the arm to the selected intake position and open the claw
                new AdjustArmToRetrievalPosition(Robot.pieceRetrievalState).schedule();
                new OpenClawCommand().schedule();
            }
        }
    }

    private void clearStates() {
        Robot.pieceRetrievalState = PieceRetrievalState.CLEAR;
        Robot.gamePieceState = GamePieceState.CLEAR;
        Robot.rowSelectionState = RowSelectionState.CLEAR;
    }
}
