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
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("default position command run");
    Robot.ARM_SUBSYSTEM.motionMagic();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.ARM_SUBSYSTEM.setArmPosition(-90, 3000, 800);
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
