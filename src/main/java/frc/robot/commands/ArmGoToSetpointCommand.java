// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.util.ArmSetpoint;

public class ArmGoToSetpointCommand extends CommandBase {
  ArmSubsystem arm = Robot.ARM_SUBSYSTEM;

  public ArmGoToSetpointCommand(ArmSetpoint setpoint) {
    addRequirements(arm);
  }

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