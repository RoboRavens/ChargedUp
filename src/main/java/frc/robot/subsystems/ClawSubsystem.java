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
    public ClawSubsystem() {
        setAndManageClawStates();
    }
    // Returns true if the sensor detects a game piece
    // And false if a game piece is not detected
    public boolean detectsGamePiece() {
        return false;
    }

    // Returns true if the claw is open
    // And false if the claw is closed
    private boolean isOpen() {
        return false;
    }

    private boolean isClosed() {
        return false;
    }

    public void open() {
        Robot.clawState = ClawState.OPENING;
    }

    public void close() {
        Robot.clawState = ClawState.CLOSING;
    }

    private void setAndManageClawStates() {
        // Sets the claw state
        new Trigger(() -> isOpen()).whileTrue(new InstantCommand(() -> Robot.clawState = ClawState.OPEN));
        new Trigger(() -> isClosed()).whileTrue(new InstantCommand(() -> Robot.clawState = ClawState.CLOSED));
        // Sets the load state
        new Trigger(() -> detectsGamePiece() && Robot.clawState == ClawState.CLOSED).whileTrue(new InstantCommand(() -> Robot.loadState = LoadState.LOADED));
        new Trigger(() -> detectsGamePiece() && Robot.clawState != ClawState.CLOSED).whileTrue(
            new InstantCommand(() -> Robot.overallState = OverallState.LOADING)
            .andThen(new CloseClawCommand())
            .andThen(new InstantCommand(() -> {
                // Check if the game piece was loaded successfully
                if (detectsGamePiece() && isClosed()) {
                    Robot.overallState = OverallState.LOADED_TRANSIT;
                }
            }))
        );
        new Trigger(() -> detectsGamePiece() == false).whileTrue(new InstantCommand(() -> Robot.loadState = LoadState.EMPTY));
    }
}
