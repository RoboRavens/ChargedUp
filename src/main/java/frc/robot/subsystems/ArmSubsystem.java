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
import frc.util.StateManagement.LoadTargetState;
import frc.util.StateManagement.OverallState;
import frc.util.StateManagement.ScoringTargetState;
import frc.util.StateManagement.ZoneState;

// TODO: Implement Arm Subsystem
public class ArmSubsystem extends SubsystemBase {
    public void setAndManageArmStates() {
        // Schedules commands that require the arm subsystem
        // The commands scheduled set the arm state in turn
        new Trigger(() -> Robot.overallState == OverallState.EJECTING).toggleOnTrue(new EjectPieceCommand());
        // Extend and rotate the arm to the loading target
        new Trigger(() -> (Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE || Robot.overallState == OverallState.GROUND_PICKUP) && Robot.loadState == LoadState.EMPTY)
        .whileTrue(new RotateArmToRetrievalPositionCommand(Robot.loadTargetState)
        .andThen(new ExtendArmToRetrievalPositionCommand(Robot.loadTargetState)).withName("Extend and rotate arm to loading target"));
        // Extend and rotate the arm to the scoring target
        new Trigger(() -> Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY && Robot.loadState == LoadState.LOADED)
        .whileTrue(new RotateArmToRowPositionCommand(Robot.scoringTargetState)
        .andThen(new ExtendArmToRowPositionCommand(Robot.scoringTargetState)).withName("Extend and rotate arm to scoring target"));
        // Retract the arm and rotate it upwards if the robot
        // - has just loaded
        // - has just scored
        // - is in a dangerous zone and does not intend to load from the ground
        new Trigger(() -> (Robot.overallState == OverallState.LOADED_TRANSIT && Robot.zoneState == ZoneState.ALLIANCE_LOADING_ZONE) 
                            || (Robot.overallState == OverallState.EMPTY_TRANSIT && Robot.zoneState == ZoneState.ALLIANCE_COMMUNITY)
                            || (isInDangerZone() && Robot.loadTargetState != LoadTargetState.GROUND))
        .whileTrue(new RetractArmCommand().andThen(new RotateArmToRowPositionCommand(ScoringTargetState.HIGH)).withName("Retract arm"));
    }

    private boolean isInDangerZone() {
        return Robot.zoneState != ZoneState.ALLIANCE_COMMUNITY && Robot.zoneState != ZoneState.ALLIANCE_LOADING_ZONE;
    }
}
