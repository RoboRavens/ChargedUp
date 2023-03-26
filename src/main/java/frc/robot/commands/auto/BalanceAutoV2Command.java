// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAutoV2Command extends SequentialCommandGroup {
  /** Creates a new BalanceAutoV2Command. */
  public BalanceAutoV2Command() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveUntilTiltedCommand(),
      new DriveUntilLevelCommand(),
      new WaitCommand(1),
      new DriveBackwardUntilLevelCommand(),
      new InstantCommand(() -> Robot.DRIVE_TRAIN_SUBSYSTEM.holdPosition())
    );
  }
}
