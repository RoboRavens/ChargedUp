package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    public boolean isInOpponentLoadingZone() {
        return false;
    }

    public boolean isInOpponentCommunity() {
        return false;
    }

    public boolean isInAllianceLoadingZone() {
        return false;
    }

    public boolean isInAllianceCommunity() {
        return false;
    }

    public boolean isAlignedWithScoringNode() {
        return false;
    }
}
