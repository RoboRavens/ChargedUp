package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RealtimeTrajectory extends SubsystemBase {

    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = networkTableInstance.getTable("Shuffleboard");
    Field2d DASHBOARD_Field2d = new Field2d();

    

    
    private double[] redAllianceColumnToYPos = new double[9];
    private double[] blueAllianceColumnToYPos = new double[9];

    public RealtimeTrajectory() {
        blueAllianceColumnToYPos[8] = 5.04;
        blueAllianceColumnToYPos[7] = 4.42;
        blueAllianceColumnToYPos[6] = 3.88;
        blueAllianceColumnToYPos[5] = 3.34;
        blueAllianceColumnToYPos[4] = 2.78;
        blueAllianceColumnToYPos[3] = 2.22;
        blueAllianceColumnToYPos[2] = 1.66;
        blueAllianceColumnToYPos[1] = 1.09;
        blueAllianceColumnToYPos[0] = 0.47;
        SmartDashboard.putData("trajectory", DASHBOARD_Field2d);
        redAllianceColumnToYPos[0] = 4.97;
        redAllianceColumnToYPos[1] = 4.38;
        redAllianceColumnToYPos[2] = 3.85;
        redAllianceColumnToYPos[3] = 3.29;
        redAllianceColumnToYPos[4] = 2.75;
        redAllianceColumnToYPos[5] = 2.15;
        redAllianceColumnToYPos[6] = 1.60;
        redAllianceColumnToYPos[7] = 1.07;
        redAllianceColumnToYPos[8] = 0.46;
            
    }
    Pose2d sideStart = new Pose2d(1.54, 2.72,Rotation2d.fromDegrees(180));  


}
