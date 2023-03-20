package frc.util.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This class holds methods relating to ChassisSpeeds
 */
public class ChassisSpeedsExtensions {
    /**
     * This method returns true if the provided ChassisSpeeds has a stopped velocity (all 3 are zero).
     * @param chassisSpeeds
     * @return True if velocity is zero, otherwise false.
     */
    public static boolean IsZero(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.omegaRadiansPerSecond == 0
            && chassisSpeeds.vxMetersPerSecond == 0
            && chassisSpeeds.vyMetersPerSecond == 0;
    }
}
