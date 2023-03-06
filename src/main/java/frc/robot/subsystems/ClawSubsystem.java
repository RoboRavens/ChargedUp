package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.claw.CloseClawCommand;
import frc.util.StateManagement.ClawState;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.OverallState;

// TODO: Implement Claw Subsystem
public class ClawSubsystem extends SubsystemBase {
    // Returns true if the sensor detects a game piece
    // And false if a game piece is not detected
    public boolean detectsGamePiece() {
        return false;
    }

    public void open() {
        Robot.clawState = ClawState.OPENING;
    }

    public void close() {
        Robot.clawState = ClawState.CLOSING;
    }

    public void setAndManageClawStates() {
        // Sets the load state
        new Trigger(() -> detectsGamePiece() && Robot.clawState == ClawState.CLOSED).whileTrue(new InstantCommand(() -> Robot.loadState = LoadState.LOADED));
        new Trigger(() -> detectsGamePiece() && Robot.clawState != ClawState.CLOSED).whileTrue(
            new InstantCommand(() -> Robot.overallState = OverallState.LOADING)
            .andThen(new CloseClawCommand())
            .andThen(new InstantCommand(() -> {
                // Check if the game piece was loaded successfully
                if (detectsGamePiece()) {
                    Robot.overallState = OverallState.LOADED_TRANSIT;
                }
            })).withName("Close claw and update overall state")
        );
        new Trigger(() -> detectsGamePiece() == false).whileTrue(new InstantCommand(() -> Robot.loadState = LoadState.EMPTY));
    }
}
