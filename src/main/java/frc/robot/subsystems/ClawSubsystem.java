package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.StateManagementNew.ClawState;
import frc.util.StateManagementNew.LoadState;

// TODO: Implement Claw Subsystem
public class ClawSubsystem extends SubsystemBase {
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

    @Override
    public void periodic() {
        // Sets the claw state
        if (isOpen()) {
            Robot.clawState = ClawState.OPEN;
        }
        else if (isClosed()) {
            Robot.clawState = ClawState.CLOSED;
        }
        // Sets the load state
        if (detectsGamePiece() && Robot.clawState == ClawState.CLOSED) {
            Robot.loadState = LoadState.LOADED;
        }
        else {
            Robot.loadState = LoadState.EMPTY;
        }
    }
}
