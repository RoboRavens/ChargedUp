// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class ArmExtendManuallyCommand extends CommandBase {
  private ArmSubsystem arm = Robot.ARM_SUBSYSTEM;
  private double voltage = Constants.ARM_MANUAL_EXTENSION_VOLTAGE;
  
  public ArmExtendManuallyCommand(boolean forward) {
    addRequirements(arm);
    if (forward == false) {
      voltage *= -1;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.brakeDisable();
    arm.setExtensionVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopExtension();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
