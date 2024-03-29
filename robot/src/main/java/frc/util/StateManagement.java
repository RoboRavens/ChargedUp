package frc.util;

public class StateManagement {
    // States set in Robot.java (mostly in the setOverallStates method)
    public enum OverallState {
        EMPTY_TRANSIT,  // "Normal" state when the robot has no game piece (aka, not actively picking one up.)
        LOADED_TRANSIT, // "Normal" state when the robot has a game piece (aka, not scoring/preparing to score, ejecting, etc.)
        GROUND_PICKUP,  // While the robot is actively attempting to collect a piece from the ground.
        SINGLE_SUBSTATION_PICKUP,   // While the robot is preparing to load a piece from the single substation.
        DOUBLE_SUBSTATION_PICKUP,   // While the robot is preparing to load a piece from the double substation.
        LOADING, // When the robot does not have a piece but detects one (while claw is closing.)
        SCORING, // When the robot has a piece, the robot is positioned to score, and the driver intends to release it (while claw opens.)
        EJECTING, // When the robot has a piece and the driver intends to release it (while any ejection actions happen, such as the claw opening.)
        PREPARING_TO_SCORE, // When the robot is loaded and in the alliance community.
        FINAL_SCORING_ALIGNMENT, // When the robot is checking final scoring conditions. If met, the game piece is ready to be released.
        ENDGAME // When the endgame override switch is flipped to true
    }

    public static boolean isManipulatingGamePiece(OverallState robotState, ClawState clawState) {
        boolean manipulatingGamePiece = false;

        if (robotState == OverallState.LOADING || robotState == OverallState.SCORING || robotState == OverallState.EJECTING || clawState == ClawState.OPENING || clawState == ClawState.CLOSING) {
            manipulatingGamePiece = true;
        }

        return manipulatingGamePiece;
    }

    // Set in the arm subsystem
    public enum ArmRotationState {
        UP,
        SINGLE_SUBSTATION,
        DOUBLE_SUBSTATION,
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
        SINGLE_SUBSTATION,
        DOUBLE_SUBSTATION,
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
        SUBSTATION,
        GROUND
    }

    // Set in the drivetrain subsystem
    public enum DrivetrainState {
        FREEHAND,   // Normal driving.
        ACTIVELY_LOADING,   // All human control locked out (briefly, while claw closes.)
        FREEHAND_WITH_ROTATION_LOCK,    // Yaw locked parallel to grid, but no effect on translation.
        ROBOT_ALIGN, // Yaw locked and sideways translation delegated to limelight, but forward/backward accessible.
        SCORING // All human control locked out (briefly, while claw opens.) (May be identical to actively loading state.)
,       GO_TO_DASHBOARD_COORDINATES
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
        NONE,
        ALLIANCE_LOADING_ZONE,
        ALLIANCE_COMMUNITY,
        ALLIANCE_CHARGE_STATION,
        NEUTRAL,
        OPPONENT_LOADING_ZONE,
        OPPONENT_COMMUNITY,
        OPPONENT_CHARGE_STATION;

        public boolean isValidZoneForSubstationAutoDrive() {
            return this == ALLIANCE_LOADING_ZONE || this == NEUTRAL;
        }

        public boolean isValidZoneForGridAutoDrive() {
            return this == ALLIANCE_COMMUNITY || this == NEUTRAL;
        }
    }
}
