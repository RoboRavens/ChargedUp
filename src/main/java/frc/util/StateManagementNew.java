package frc.util;

public class StateManagementNew {
    // States set in Robot.java (mostly in the setOverallStates method)
    public enum OverallState {
        EMPTY_TRANSIT,
        LOADED_TRANSIT,
        GROUND_PICKUP,
        HPS_PICKUP,
        LOADING, // When the robot does not have a piece but detects one
        SCORING, // When the robot has a piece, the robot is positioned to score, and the driver intends to release it
        EJECTING, // When the robot has a piece and the driver intends to release it
        PREPARING_TO_SCORE, // When the robot is loaded and in the alliance community
        FINAL_SCORING_ALIGNMENT // When the robot is
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
        FREEHAND,
        HPS_ALIGN,
        ACTIVELY_LOADING,
        FREEHAND_WITH_ROTATION_LOCK,
        FINAL_SCORING_ROTATION_LOCK_AND_AUTO_ALIGN,
        SCORING
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
}
