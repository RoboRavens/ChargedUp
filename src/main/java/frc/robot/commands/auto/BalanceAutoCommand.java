// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.DrivetrainChargeStationBalancingCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAutoCommand extends SequentialCommandGroup {
  /** Creates a new BalanceAutoCommand. */
  public BalanceAutoCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveUntilBridgeCommand(),
      new WaitCommand(0.5),
      new InstantCommand(() -> Robot.DRIVE_TRAIN_SUBSYSTEM.drive(new ChassisSpeeds(2, 0, 0))),
      new DrivetrainChargeStationBalancingCommand()
    );
  }
}
