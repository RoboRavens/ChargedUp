package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmGoToSetpointDangerousCommand;
import frc.robot.commands.arm.ArmSequencedRetractionCommand;
import frc.robot.commands.claw.AutoClawOpenCommand;
import frc.robot.commands.claw.ClawOpenCommand;
import frc.robot.commands.drivetrain.DrivetrainChargeStationBalancingCommand;

public class PreloadBalanceAutoCommand extends CommandBase {
    public static Command getAutoMode(AutoEnums.AutoAlliance autoAlliance) {
        String pathFile = "Preload Cone + Balance " + autoAlliance.toString();
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathFile, new PathConstraints(1.6, 0.6));

        PathPlannerTrajectory path1 = path.get(0);
        HashMap<String, Command> pathEventMap1 = new HashMap<>();

        FollowPathWithEvents path1WithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(path1), 
            path1.getMarkers(),
            pathEventMap1);

        Command scoreTwoAndBalanceCommand = 
        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(path1)
        // .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_OPPOSITE_SETPOINT)).withTimeout(3)
        // .andThen(new ClawOpenCommand().withTimeout(3))
        // .andThen(new ArmSequencedRetractionCommand()).withTimeout(6)
        .andThen(path1WithEvents)
        .andThen(new DrivetrainChargeStationBalancingCommand()); 

        return scoreTwoAndBalanceCommand;
    }
}
