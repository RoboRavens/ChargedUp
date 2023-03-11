package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class RumbleCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  
   @Override
   public void initialize() {
     m_timer.start();
     Robot.GAMEPAD.setRumbleOn();
  }

  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    Robot.GAMEPAD.setRumbleOff();
  }

    @Override
    public boolean isFinished() {
        boolean isFinished = false;

        if (m_timer.get() >= Constants.RUMBLE_TIME) {
            isFinished = true;
        }
        return isFinished;
    }
}


