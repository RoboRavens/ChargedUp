package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.util.StateManagement.LoadTargetState;

public class ArmRotateToRetrievalPositionCommand extends CommandBase {
    
    public ArmRotateToRetrievalPositionCommand(LoadTargetState ground) {
        addRequirements(Robot.ARM_SUBSYSTEM);
    }

    // TODO: Implement this command
    // Remember to update the arm state
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.ARM_SUBSYSTEM.motionMagic();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot.ARM_SUBSYSTEM.setArmRotationAngularPosition(90, 3000, 800);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
