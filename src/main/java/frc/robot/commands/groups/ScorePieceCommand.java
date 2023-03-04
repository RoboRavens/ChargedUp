package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.RetractArmCommand;
import frc.robot.commands.claw.OpenClawCommand;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.PieceState;
import frc.util.StateManagement.ScoringTargetState;

public class ScorePieceCommand extends SequentialCommandGroup {
    public ScorePieceCommand() {
        addRequirements(Robot.CLAW_SUBSYSTEM, Robot.ARM_SUBSYSTEM);
        addCommands(
            new InstantCommand(() -> Robot.overallState = OverallState.SCORING),
            // new OpenClawCommand(),
            new WaitCommand(3),
            new ParallelCommandGroup(
                // new DrivetrainDefaultCommand(),
                // new RetractArmCommand(),
                new InstantCommand(() -> Robot.overallState = OverallState.EMPTY_TRANSIT),
                new InstantCommand(() -> Robot.pieceState = PieceState.NONE),
                new InstantCommand(() -> Robot.scoringTargetState = ScoringTargetState.NONE)
            )
        );
    }
}
