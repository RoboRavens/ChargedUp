// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveBackwardUntilLevelCommand extends CommandBase {
  /** Creates a new BalanceAutoCommand. */
  public DriveBackwardUntilLevelCommand() {
    addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.DRIVE_TRAIN_SUBSYSTEM.drive(
      new ChassisSpeeds(
        .5,
        0,
        0
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.DRIVE_TRAIN_SUBSYSTEM.drive(
      new ChassisSpeeds(
        0,
        0,
        0
      )
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isTilted = (Math.abs(Robot.DRIVE_TRAIN_SUBSYSTEM.getRoll()) <= Constants.ROBOT_IS_LEVEL_DEGREES);
    return isTilted;
  }
}
