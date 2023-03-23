package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClawSubsystem;
import frc.util.StateManagement.ClawState;

public class ClawCloseCommand extends CommandBase {
    Timer timer = new Timer();
    ClawSubsystem claw = Robot.CLAW_SUBSYSTEM;

    public ClawCloseCommand() {
        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        claw.close();
        claw.setClawState(ClawState.CLOSING);
        timer.reset();
        timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        claw.setClawState(ClawState.CLOSED);
        timer.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isFinished = false;

        if (timer.get() >= Constants.CLAW_CLOSE_TIMEOUT_SECONDS) {
            isFinished = true;
        }

        return isFinished;
    }
}
