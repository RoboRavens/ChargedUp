package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.StateManagement.ClawState;

public class CloseClawCommand extends CommandBase {
    public CloseClawCommand() {
        addRequirements(Robot.CLAW_SUBSYSTEM);
    }

    @Override
    public void execute() {
        Robot.clawState = ClawState.CLOSING;
        // TODO: Implement this command
    }
    
    @Override
    public void end(boolean interrupted) {
        Robot.clawState = ClawState.CLOSED;
    }

    @Override
    public boolean isFinished() {
        // Pauses the program for a second (for testing purposes)
        try {
            Thread.sleep(1000);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        return true;
    }
}
