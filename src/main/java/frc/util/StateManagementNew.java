package frc.util;

public class StateManagementNew {
    // States set in Robot.java (mostly in the setOverallStates method)
    public enum OverallState {
        EMPTY_TRANSIT,  // "Normal" state when the robot has no game piece (aka, not actively picking one up.)
        LOADED_TRANSIT, // "Normal" state when the robot has a game piece (aka, not scoring/preparing to score, ejecting, etc.)
        GROUND_PICKUP,  // While the robot is actively attempting to collect a piece from the ground.
        HPS_PICKUP,     // While the robot is loading a piece from the HPS.
        LOADING, // When the robot does not have a piece but detects one (while claw is closing.)
        SCORING, // When the robot has a piece, the robot is positioned to score, and the driver intends to release it (while claw opens.)
        EJECTING, // When the robot has a piece and the driver intends to release it (while any ejection actions happen, such as the claw opening.)
        PREPARING_TO_SCORE, // When the robot is loaded and in the alliance community.
        FINAL_SCORING_ALIGNMENT // When the robot is checking final scoring conditions. If met, the game piece is ready to be released.
    }

    // Set in the arm subsystem
    public enum ArmRotationState {
        UP,
        HPS,
        SCORE_HIGH,
        SCORE_MID,
        SCORE_LOW,
        SCORE_STAGING,
        COLLECT_GROUND,
        EJECT,
        ACTIVELY_LOADING,
        ACTIVELY_SCORING,
        ACTIVELY_EJECTING
    }

    // Set in the arm subsystem
    public enum ArmExtensionState {
        RETRACTED,
        HPS,
        SCORE_HIGH,
        SCORE_MID,
        SCORE_LOW,
        SCORE_STAGING,
        COLLECT_GROUND,
        EJECT,
        ACTIVELY_LOADING,
        ACTIVELY_SCORING,
        ACTIVELY_EJECTING
    }

    // Set in configureButtonBindings() in Robot.java
    public enum PieceState {
        NONE,
        CONE,
        CUBE
    }

    // Set in the periodic method of the claw subsystem
    public enum LoadState {
        EMPTY,
        LOADED
    }

    // Set in configureButtonBindings() in Robot.java
    public enum LoadTargetState {
        HPS,
        GROUND
    }

    // Set in the drivetrain subsystem
    public enum DrivetrainState {
        FREEHAND,   // Normal driving.
        HPS_ALIGN,  // Yaw locked parallel to HPS, and translation auto-aligns to shelf.
        ACTIVELY_LOADING,   // All human control locked out (briefly, while claw closes.)
        FREEHAND_WITH_ROTATION_LOCK,    // Yaw locked parallel to grid, but no effect on translation.
        FINAL_SCORING_ROTATION_LOCK_AND_AUTO_ALIGN, // Yaw locked and sideways translation delegated to limelight, but forward/backward accessible.
        SCORING // All human control locked out (briefly, while claw opens.) (May be identical to actively loading state.)
    }

    // Set in the claw subsystem periodic and open/close methods
    public enum ClawState {
        OPEN,
        CLOSED,
        OPENING,
        CLOSING
    }

    // TODO: set these states in the limelight subsystem
    public enum LimelightState {
        TAG_TRACKING,
        TAPE_TRACKING
    }
    
    // Set in configureButtonBindings() in Robot.java
    public enum ScoringTargetState {
        NONE,
        LOW,
        MID,
        HIGH
    }

    public enum ZoneState {
        ALLIANCE_LOADING_ZONE,
        ALLIANCE_COMMUNITY,
        ALLIANCE_BRIDGE,
        NEUTRAL,
        OPPONENT_LOADING_ZONE,
        OPPONENT_COMMUNITY,
        OPPONENT_BRIDGE
    }
}
