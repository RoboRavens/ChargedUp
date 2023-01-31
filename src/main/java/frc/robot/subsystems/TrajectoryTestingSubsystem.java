package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class TrajectoryTestingSubsystem {
   
    public TrajectoryTestingSubsystem() {
        _Odometry = new SwerveDriveOdometry(m_kinematics, getGyroRotation(), null);
        m_field.setRobotPose(_Odometry.getPoseMeters());
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putString("This is a test", "This is a test");
    }
   
    private final Field2d m_field = new Field2d();
    

    private Rotation2d getGyroRotation() {
        return new Rotation2d();
        
    }

    
  private final SwerveDriveOdometry _Odometry;
 
  

// Creating my kinematics object using the module locations
SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()
);




// Creating my odometry object from the kinematics object and the initial wheel positions.
// Here, our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing the opposing alliance wall.
SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  m_kinematics, getGyroRotation(),
  new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()

  }, new Pose2d(5.0, 13.5, new Rotation2d()));

}
