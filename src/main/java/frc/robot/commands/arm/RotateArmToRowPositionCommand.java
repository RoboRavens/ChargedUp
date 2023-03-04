package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.util.StateManagement.ScoringTargetState;

public class RotateArmToRowPositionCommand extends CommandBase {

    public RotateArmToRowPositionCommand(ScoringTargetState scoringTargetState) {
        addRequirements(Robot.ARM_SUBSYSTEM);
    }

    @Override
    public void execute() {

    }

    // TODO: Implement this command
    // Remember to update the arm rotation state
}
