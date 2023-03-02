package frc.robot.subsystems;

import java.lang.reflect.Array;

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

    private Rotation2d Rotation2d;

    public SwerveModulePosition m_frontLeftPosition = new SwerveModulePosition(0.0,Rotation2d);

    public SwerveModulePosition m_frontRightPosition = new SwerveModulePosition(0.0,Rotation2d);

    public SwerveModulePosition m_backLeftPosition  = new SwerveModulePosition(0.0, Rotation2d);

    public SwerveModulePosition m_backRightPosition = new SwerveModulePosition(0.0,Rotation2d);

    public TrajectoryTestingSubsystem() {
        _Odometry = new SwerveDriveOdometry(m_kinematics, getGyroRotation(), getSwerveModulePositions());
        m_field.setRobotPose(_Odometry.getPoseMeters());
       // SmartDashboard.putString("This is a test", "This is a test");

    }

    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final Field2d m_field = new Field2d();

    private Rotation2d getGyroRotation() {
        return new Rotation2d();

    }

    private final SwerveDriveOdometry _Odometry;

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
         m_backRightLocation, m_backLeftLocation, m_frontLeftLocation, m_frontRightLocation);

    // Creating my odometry object from the kinematics object and the initial wheel
    // positions.
    // Here, our starting pose is 5 meters along the long end of the field and in
    // the
    // center of the field along the short end, facing the opposing alliance wall.
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics, getGyroRotation(),
            getSwerveModulePositions(),
            new Pose2d(5.0, 13.5, new Rotation2d()));

            public SwerveModulePosition[] getSwerveModulePositions() {
                return new SwerveModulePosition[] {
                    m_frontLeftPosition,
   
                    m_frontRightPosition,
               
                    m_backLeftPosition,
               
                    m_backRightPosition
   
               };
           
            }
}
