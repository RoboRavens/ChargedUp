package frc.robot.commands.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToSetpointDangerousCommand;
import frc.robot.commands.claw.AutoClawOpenCommand;
import frc.robot.commands.drivetrain.ChargeStationBalancingCommand;
import frc.util.AutoMode;

public class PreloadAndBalanceBlueCommand {
    public static AutoMode getAutoMode() {
        PathPlannerTrajectory scoringToBridgeTrajectory = PathPlanner.loadPath("Preload and Balance", new PathConstraints(1, 0.4));

        HashMap<String, Command> eventMap = new HashMap<>();

        FollowPathWithEvents scoringToBridgePath = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(scoringToBridgeTrajectory), 
            scoringToBridgeTrajectory.getMarkers(), 
            eventMap);

        Command preloadAndBalanceCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(scoringToBridgeTrajectory)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT))
        .andThen(new AutoClawOpenCommand())
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_FULL_RETRACT_SETPOINT))
        .andThen(scoringToBridgePath)
        .andThen(new ChargeStationBalancingCommand());

        return new AutoMode("Preload and Balance", preloadAndBalanceCommand);
    }
}
