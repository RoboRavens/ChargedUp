package frc.util;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleConverter {
    public static SwerveModuleState ToSwerveModuleState(SwerveModule module, double angleOffsetInDegrees) {
        return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle() + Math.toRadians(angleOffsetInDegrees)));
    }
}