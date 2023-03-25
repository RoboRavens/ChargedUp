package frc.robot.commands.auto;

import java.util.HashMap;

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

public class PreloadMobilityAndBalanceAutoCommand extends CommandBase {
    public static Command getAutoMode(AutoEnums.AutoAlliance autoAlliance) {
        String pathFile = "Preload Cone + Mobility + Balance " + autoAlliance.toString();
        PathPlannerTrajectory path = PathPlanner.loadPath(pathFile, new PathConstraints(1.6, 0.6));
        
        HashMap<String, Command> pathEventMap = new HashMap<>();

        FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
            Robot.DRIVE_TRAIN_SUBSYSTEM.CreateFollowTrajectoryCommandSwerveOptimized(path), 
            path.getMarkers(),
            pathEventMap);

        Command pathCommand = pathWithEvents;


        Command scoreTwoAndBalanceCommand = 
        Robot.DRIVE_TRAIN_SUBSYSTEM.CreateSetOdometryToTrajectoryInitialPositionCommand(path)
        .andThen(new ArmGoToSetpointDangerousCommand(Constants.ARM_SCORE_CONE_HIGH_SETPOINT)).withTimeout(3)
        .andThen(new ClawOpenCommand().withTimeout(3))
        .andThen(new InstantCommand(() -> SmartDashboard.putNumber("Before retraction command", Timer.getFPGATimestamp())))
        .andThen(new ArmSequencedRetractionCommand()).withTimeout(6)
        .andThen(new InstantCommand(() -> SmartDashboard.putNumber("After retraction command", Timer.getFPGATimestamp())))
        .andThen(pathCommand).withTimeout(10)
        .andThen(new DrivetrainChargeStationBalancingCommand()); 

        return scoreTwoAndBalanceCommand;
    }
}
