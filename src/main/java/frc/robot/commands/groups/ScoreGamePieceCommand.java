package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.robot.commands.claw.OpenClawCommand;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;

public class ScoreGamePieceCommand extends SequentialCommandGroup {
    public ScoreGamePieceCommand() {
        addCommands(
            new OpenClawCommand(),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                new DrivetrainDefaultCommand(),
                new RetractArmCommand()
            )
        );
    }
}
