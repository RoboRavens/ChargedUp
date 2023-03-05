package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.StateManagementNew.ClawState;

public class OpenClawCommand extends CommandBase {
    public OpenClawCommand() {
        addRequirements(Robot.CLAW_SUBSYSTEM);
    }

    @Override
    public void initialize() {
        initialize();
    }

new 
    public void () {
    Robot.clawState = ClawState.OPEN;
    }

    // TODO: Implement this command
    // Remember to update claw state
}
