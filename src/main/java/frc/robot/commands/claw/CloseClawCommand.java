package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CloseClawCommand extends CommandBase {
    public CloseClawCommand() {
        addRequirements(Robot.CLAW_SUBSYSTEM);
    }
}
