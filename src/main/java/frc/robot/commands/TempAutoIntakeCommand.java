package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TempAutoIntakeCommand extends CommandBase {
    public TempAutoIntakeCommand() {
        addRequirements(Robot.TEMP_CONVEYANCE_SUBYSTEM);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    @Override
    public void execute() {
        Robot.TEMP_CONVEYANCE_SUBYSTEM.setConveyanceIntakeCargo();
    }
    @Override
    public void end(boolean interrupted) {
        Robot.TEMP_CONVEYANCE_SUBYSTEM.stopConveyanceOne();
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}
