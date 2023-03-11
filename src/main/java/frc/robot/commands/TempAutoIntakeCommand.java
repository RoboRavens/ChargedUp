package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TempAutoIntakeCommand extends CommandBase {
    Timer _timer = new Timer();
    public TempAutoIntakeCommand() {
        addRequirements(Robot.TEMP_CONVEYANCE_SUBYSTEM);
    }
    @Override
    public void initialize() {
        _timer.restart();
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
        if (_timer.get() > 1) {
            return true;
        }
        return false;
    }
}
