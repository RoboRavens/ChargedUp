package frc.robot.shuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightInformation {
    LimelightSubsystem limelightOne = Robot.LIMELIGHT_SUBSYSTEM_ONE;
    LimelightSubsystem limelightTwo = Robot.LIMELIGHT_SUBSYSTEM_TWO;
    private final GenericEntry _getTv;
    private final GenericEntry _getTa;
    private final GenericEntry _getTv2;
    private final GenericEntry _getTa2;
   
  public void limelightShuffleboard() {
        ShuffleboardTab limelightTab = Shuffleboard.getTab("limelight");
        _getTa = limelightTab.add("limelight Area LL 1", 0.0).withPosition(8, 0).withSize(1, 1).getEntry();
        _getTa2 = limelightTab.add("limelight Area LL 2", 0.0).withPosition(8, 0).withSize(1, 1).getEntry();
        _getTv = limelightTab.add("limelight Area LL 1", 0.0).withPosition(8, 0).withSize(1, 1).getEntry();
        _getTv2 = limelightTab.add("limelight Area LL 2", 0.0).withPosition(8, 0).withSize(1, 1).getEntry();
    
    }



    public void updateLimelightInformation(LimelightSubsystem limelightOne, LimelightSubsystem limelightTwo) {
       _getTv.setDouble(limelightOne.hasVisionTarget());
       _getTa.setDouble(limelightOne.getTa());
       _getTa2.setDouble(limelightTwo.getTa());
       _getTv.setDouble(limelightTwo.getTa());
        
    }
}
