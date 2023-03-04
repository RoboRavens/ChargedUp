package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.arm.RotateArmToRetrievalPositionCommand;
import frc.robot.commands.arm.RotateArmToRowPositionCommand;
import frc.robot.commands.groups.EjectPieceCommand;
import frc.robot.commands.arm.ExtendArmToRetrievalPositionCommand;
import frc.robot.commands.arm.ExtendArmToRowPositionCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.util.StateManagement.LoadState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.ZoneState;

// TODO: Implement Arm Subsystem
public class ArmSubsystem extends SubsystemBase {
    
    public ArmSubsystem() {
        setAndManageArmStates();
    }

    @Override
    public void periodic() {}

    private void setAndManageArmStates() {
        // Schedules commands that require the arm subsystem
        // The commands scheduled set the arm state in turn
        new Trigger(() -> Robot.overallState == OverallState.EJECTING).whileTrue(new EjectPieceCommand());
        // Extend and rotate the arm to the loading target
        new Trigger(() -> Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE || Robot.overallState == OverallState.GROUND_PICKUP)
        .whileTrue(new RotateArmToRetrievalPositionCommand(Robot.loadTargetState)
        .andThen(new ExtendArmToRetrievalPositionCommand(Robot.loadTargetState)));
        // Extend and rotate the arm to the scoring target
        new Trigger(() -> Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY)
        .whileTrue(new RotateArmToRowPositionCommand(Robot.scoringTargetState)
        .andThen(new ExtendArmToRowPositionCommand(Robot.scoringTargetState)));
        // Adjust the arm rotation to the target scoring row position when the robot is loaded and in the neutral zone
        new Trigger(() -> Robot.zoneState == ZoneState.NEUTRAL && Robot.loadState == LoadState.LOADED)
        .whileTrue(new RetractArmCommand().andThen(new RotateArmToRowPositionCommand(Robot.scoringTargetState)));
        // Adjust the arm rotation to the target retrieval position when the robot is not loaded and in the neutral zone
        new Trigger(() -> Robot.zoneState == ZoneState.NEUTRAL && Robot.loadState == LoadState.LOADED)
        .whileTrue(new RetractArmCommand().andThen(new RotateArmToRetrievalPositionCommand(Robot.loadTargetState)));
    }
}
