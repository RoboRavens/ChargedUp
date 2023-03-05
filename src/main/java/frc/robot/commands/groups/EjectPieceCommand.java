package frc.robot.commands.groups;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.RotateArmToRetrievalPositionCommand;
import frc.robot.commands.arm.ExtendArmToRetrievalPositionCommand;
import frc.robot.commands.arm.ExtendArmToRowPositionCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.robot.commands.claw.OpenClawCommand;
import frc.util.StateManagement.ArmRotationState;
import frc.util.StateManagement.LoadTargetState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.PieceState;
import frc.util.StateManagement.ScoringTargetState;

public class EjectPieceCommand extends SequentialCommandGroup {
    public EjectPieceCommand() {
        addRequirements(Robot.ARM_SUBSYSTEM, Robot.CLAW_SUBSYSTEM);
        // TODO: set the arm rotation and extension state
        addCommands(
            // new RotateArmToRetrievalPositionCommand(LoadTargetState.GROUND),
            // new ExtendArmToRetrievalPositionCommand(LoadTargetState.GROUND),
            // new OpenClawCommand(),
            new WaitCommand(3),
            // new RetractArmCommand(),
            new InstantCommand(() -> Robot.overallState = OverallState.EMPTY_TRANSIT),
            new InstantCommand(() -> Robot.pieceState = PieceState.NONE),
            new InstantCommand(() -> Robot.scoringTargetState = ScoringTargetState.NONE)
        );
    }
}
