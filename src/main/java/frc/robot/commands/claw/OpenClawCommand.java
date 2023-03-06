package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.StateManagement.ClawState;

public class OpenClawCommand extends CommandBase {
    public OpenClawCommand() {
        addRequirements(Robot.CLAW_SUBSYSTEM);
    }

    @Override
    public void execute() {
        Robot.clawState = ClawState.OPENING;
        // TODO: Implement this command
    }

    @Override
    public void end(boolean interrupted) {
        Robot.clawState = ClawState.OPEN;
    }
    
    @Override
    public boolean isFinished() {
        try {
            Thread.sleep(1000);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        return true;
    }
}
