package frc.util.field;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldMeasurements {
    // Field measurements in inches.
    // There is a .03 inch discrepancy in the field measurements, likely due to a rounding error on FIRST's part.
    // Half the field as measured by its components, multiplied by two, is .03 inches narrower than the overall field.
    // That comes out to .762 millimeters, so not enough to worry about.
    public static final double OVERALL_FIELD_WIDTH_INCHES = 651.25;
    public static final double HALF_FIELD_WIDTH_INCHES = 325.61;
    public static final double HALF_FIELD_WIDTH_TIMES_2_INCHES = HALF_FIELD_WIDTH_INCHES * 2;

    // This field height value is slightly shorter than the nominally given height of 26 foot 3.5 inches.
    // This is compiled by the various zones. The discrepancy could be due to the inclusion of the field wall.
    public static final double FIELD_HEIGHT_INCHES = (59.39 * 2) + 99.07 + 96;

    // Field elements used as references.
    public static final double DOUBLE_SUBSTATION_WIDTH_INCHES = 14;
    public static final double LOADING_ZONE_WIDE_AREA_WIDTH_PAST_DOUBLE_SUBSTATION_INCHES = 118.25;
    
    // Community.
    public static final double DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES = 54.25;
    public static final double COMMUNITY_LENGTH_INCHES = 216.5;
    public static final double COMMUNITY_NORTHERN_SECTION_WIDTH_INCHES = 78; // 132.25 - 54.25.
    public static final double COMMUNITY_NORTHERN_SECTION_HEIGHT_INCHES = 59.39;
    public static final double COMMUNITY_COOP_SECTION_WIDTH_INCHES = 60.69;
    public static final double COMMUNITY_COOP_SECTION_HEIGHT_INCHES = 96; // 8 feet, same as the charge station.
    public static final double COMMUNITY_SOUTHERN_SECTION_WIDTH_INCHES = 138.87; // 224 minus 85.13, from the layout guide.
    public static final double COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES = 59.39;

    // Charge station, which is also part of the community.
    public static final double COMMUNITY_CHARGE_STATION_WIDTH_INCHES = 76.125; // 6 foot 4 and a quarter.
    public static final double COMMUNITY_CHARGE_STATION_HEIGHT_INCHES = 96; // 8 feet, same as the charge station.

    // Community southwest corner coordinates.
    public static final double COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_X_INCHES = DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES;
    public static final double COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_Y_INCHES = COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES + COMMUNITY_COOP_SECTION_HEIGHT_INCHES;
    public static final double COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_X_INCHES = DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES;
    public static final double COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_Y_INCHES = COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES;
    public static final double COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_X_INCHES = DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES;
    public static final double COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_Y_INCHES = 0;
    public static final double COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_X_INCHES = DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES + COMMUNITY_COOP_SECTION_WIDTH_INCHES;
    public static final double COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_Y_INCHES = COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES;

    // Loading zone.
    public static final double LOADING_ZONE_WIDE_AREA_WIDTH_INCHES = DOUBLE_SUBSTATION_WIDTH_INCHES + LOADING_ZONE_WIDE_AREA_WIDTH_PAST_DOUBLE_SUBSTATION_INCHES;
    public static final double LOADING_ZONE_WIDE_AREA_HEIGHT_INCHES = 99.07;
    public static final double LOADING_ZONE_NARROW_AREA_WIDTH_INCHES = HALF_FIELD_WIDTH_INCHES - LOADING_ZONE_WIDE_AREA_WIDTH_INCHES - 61.36;
    public static final double LOADING_ZONE_NARROW_AREA_HEIGHT_INCHES = 50.5;

    // Loading zone southwest corner coordinates.
    public static final double LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_X_INCHES = 0;
    public static final double LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_Y_INCHES = COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_Y_INCHES + COMMUNITY_NORTHERN_SECTION_HEIGHT_INCHES;
    public static final double LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_X_INCHES = LOADING_ZONE_WIDE_AREA_WIDTH_INCHES;
    public static final double LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_Y_INCHES = FIELD_HEIGHT_INCHES - LOADING_ZONE_NARROW_AREA_HEIGHT_INCHES;

    // For ease of use in other places, the net height of the wide LZ area, without the narrow height.
    public static final double LOADING_ZONE_WIDE_AREA_MARGINAL_HEIGHT_INCHES = LOADING_ZONE_WIDE_AREA_HEIGHT_INCHES - LOADING_ZONE_NARROW_AREA_HEIGHT_INCHES;

    // Neutral zone. In order to avoid error stackup, and for general symmetry across sides,
    // we'll cut the neutral zone in half based on the center line.
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_WIDTH_INCHES = 61.36;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_INCHES = 50.5;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_WIDTH_INCHES = HALF_FIELD_WIDTH_INCHES - COMMUNITY_NORTHERN_SECTION_WIDTH_INCHES - DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES;
    
    // Height of the northern section of the community plus MINUS TWO
    // because the community includes the tape but the neutral zone does not,
    // plus the marginal height of the wide section of the loading zone.
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_HEIGHT_INCHES = 
        COMMUNITY_NORTHERN_SECTION_HEIGHT_INCHES - 2 +
        LOADING_ZONE_WIDE_AREA_MARGINAL_HEIGHT_INCHES;
    
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_WIDTH_INCHES = 132.49; // 85.13 + 47.36, from the layout guide.
    
    // Height of the southern section of the community PLUS the bridge,
    // PLUS TWO because the tape on the northern side of the bridge is included in the NZ
    // but it's not included in the bridge calculation here.
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_HEIGHT_INCHES =
        COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES + 
        COMMUNITY_CHARGE_STATION_HEIGHT_INCHES + 2;
    
    // Neutral zone southwest corner coordinates.
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_X_INCHES = LOADING_ZONE_WIDE_AREA_WIDTH_INCHES + LOADING_ZONE_NARROW_AREA_WIDTH_INCHES;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_Y_INCHES = FIELD_HEIGHT_INCHES - NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_INCHES;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_X_INCHES = DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES + COMMUNITY_NORTHERN_SECTION_WIDTH_INCHES;
    // We don't include the two inches for the tape outside of the bridge in the community, but we'll include for the neutral zone - better safe than sorry.
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_Y_INCHES = COMMUNITY_COOP_SECTION_HEIGHT_INCHES + COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES + 2;
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_X_INCHES = DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES + COMMUNITY_SOUTHERN_SECTION_WIDTH_INCHES;
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_Y_INCHES = 0;
    
    // AprilTags.
    // The basis for AprilTag heights is tags 7 (blue) and 2 (red) which are centered upon the center of the bridge.
    public static final double APRILTAG_2_7_HEIGHT_INCHES = COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES + (COMMUNITY_CHARGE_STATION_HEIGHT_INCHES / 2);
    public static final double APRILTAG_1_8_HEIGHT_INCHES = APRILTAG_2_7_HEIGHT_INCHES - 66;
    public static final double APRILTAG_3_6_HEIGHT_INCHES = APRILTAG_2_7_HEIGHT_INCHES + 66;
    public static final double APRILTAG_4_5_HEIGHT_INCHES = APRILTAG_3_6_HEIGHT_INCHES + 91.55;

    // Tags 4 and 5 are on the double substation wall, and the other tags are all a fixed offset past that.
    public static final double APRILTAG_4_5_WIDTH_INCHES = DOUBLE_SUBSTATION_WIDTH_INCHES;
    public static final double APRILTAG_123_678_WIDTH_INCHES = APRILTAG_4_5_WIDTH_INCHES + 26.19;

    // Calculate each individual tag's width coordinates. Now the difference between blue and red matters.
    // The blue tags exactly math the general coordinates, but the red ones have to be inverted.
    public static double APRILTAG_123_WIDTH_INCHES = HALF_FIELD_WIDTH_TIMES_2_INCHES - APRILTAG_123_678_WIDTH_INCHES;
    public static double APRILTAG_1_WIDTH_INCHES = APRILTAG_123_WIDTH_INCHES;
    public static double APRILTAG_2_WIDTH_INCHES = APRILTAG_123_WIDTH_INCHES;
    public static double APRILTAG_3_WIDTH_INCHES = APRILTAG_123_WIDTH_INCHES;
    public static double APRILTAG_4_WIDTH_INCHES = HALF_FIELD_WIDTH_TIMES_2_INCHES - APRILTAG_4_5_WIDTH_INCHES;
    public static double APRILTAG_5_WIDTH_INCHES = APRILTAG_4_5_WIDTH_INCHES;
    public static double APRILTAG_6_WIDTH_INCHES = APRILTAG_123_678_WIDTH_INCHES;
    public static double APRILTAG_7_WIDTH_INCHES = APRILTAG_123_678_WIDTH_INCHES;
    public static double APRILTAG_8_WIDTH_INCHES = APRILTAG_123_678_WIDTH_INCHES;

    // Height needs no inversion since its constant between the alliance colors.
    public static double APRILTAG_1_HEIGHT_INCHES = APRILTAG_1_8_HEIGHT_INCHES;
    public static double APRILTAG_2_HEIGHT_INCHES = APRILTAG_2_7_HEIGHT_INCHES;
    public static double APRILTAG_3_HEIGHT_INCHES = APRILTAG_3_6_HEIGHT_INCHES;
    public static double APRILTAG_4_HEIGHT_INCHES = APRILTAG_4_5_HEIGHT_INCHES;
    public static double APRILTAG_5_HEIGHT_INCHES = APRILTAG_4_5_HEIGHT_INCHES;
    public static double APRILTAG_6_HEIGHT_INCHES = APRILTAG_3_6_HEIGHT_INCHES;
    public static double APRILTAG_7_HEIGHT_INCHES = APRILTAG_2_7_HEIGHT_INCHES;
    public static double APRILTAG_8_HEIGHT_INCHES = APRILTAG_1_8_HEIGHT_INCHES;

    // Field measurements in meters, which is the native unit for PathPlanner coordinates.
    public static final double INCHES_TO_METERS_CONVERSION = .0254;

    // Main field measurements.
    public static final double HALF_FIELD_WIDTH_TIMES_2_METERS = HALF_FIELD_WIDTH_TIMES_2_INCHES * INCHES_TO_METERS_CONVERSION;

    // Community.
    public static final double DISTANCE_FROM_WALL_TO_COMMUNITY_METERS = DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_LENGTH_METERS = COMMUNITY_LENGTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_NORTHERN_SECTION_WIDTH_METERS = COMMUNITY_NORTHERN_SECTION_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_NORTHERN_SECTION_HEIGHT_METERS = COMMUNITY_NORTHERN_SECTION_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_COOP_SECTION_WIDTH_METERS = COMMUNITY_COOP_SECTION_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_COOP_SECTION_HEIGHT_METERS = COMMUNITY_COOP_SECTION_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_SOUTHERN_SECTION_WIDTH_METERS = COMMUNITY_SOUTHERN_SECTION_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_SOUTHERN_SECTION_HEIGHT_METERS = COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;

    // Charge station, which is also part of the community.
    public static final double COMMUNITY_CHARGE_STATION_WIDTH_METERS = COMMUNITY_CHARGE_STATION_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_CHARGE_STATION_HEIGHT_METERS = COMMUNITY_CHARGE_STATION_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    
    
    // Community southwest corner coordinates.
    public static final double COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_X_METERS = COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_Y_METERS = COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_X_METERS = COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_Y_METERS = COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_X_METERS = COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_Y_METERS = COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_X_METERS = COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_Y_METERS = COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;

    // Loading zone.
    public static final double LOADING_ZONE_WIDE_AREA_WIDTH_METERS = LOADING_ZONE_WIDE_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_WIDE_AREA_HEIGHT_METERS = LOADING_ZONE_WIDE_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_NARROW_AREA_WIDTH_METERS = LOADING_ZONE_NARROW_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_NARROW_AREA_HEIGHT_METERS = LOADING_ZONE_NARROW_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;

    // Loading zone southwest corner coordinates.
    public static final double LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_X_METERS = LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_Y_METERS = LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_X_METERS = LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_Y_METERS = LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;

    // Neutral zone.
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_WIDTH_METERS = NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_METERS = NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_WIDTH_METERS = NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_HEIGHT_METERS = NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_WIDTH_METERS = NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_HEIGHT_METERS = NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;

    // Neutral zone southwest corner coordinates.
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_X_METERS = NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_Y_METERS = NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_X_METERS = NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_Y_METERS = NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_X_METERS = NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_X_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_Y_METERS = NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_Y_INCHES * INCHES_TO_METERS_CONVERSION;

    // AprilTags.
    public static final double APRILTAG_1_WIDTH_METERS = APRILTAG_1_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_1_HEIGHT_METERS = APRILTAG_1_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_2_WIDTH_METERS = APRILTAG_2_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_2_HEIGHT_METERS = APRILTAG_2_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_3_WIDTH_METERS = APRILTAG_3_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_3_HEIGHT_METERS = APRILTAG_3_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_4_WIDTH_METERS = APRILTAG_4_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_4_HEIGHT_METERS = APRILTAG_4_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_5_WIDTH_METERS = APRILTAG_5_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_5_HEIGHT_METERS = APRILTAG_5_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_6_WIDTH_METERS = APRILTAG_6_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_6_HEIGHT_METERS = APRILTAG_6_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_7_WIDTH_METERS = APRILTAG_7_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_7_HEIGHT_METERS = APRILTAG_7_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_8_WIDTH_METERS = APRILTAG_8_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double APRILTAG_8_HEIGHT_METERS = APRILTAG_8_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;

    // width = 30.25 inch


    // length = 34.5
    public static final Translation2d[] RED_NODE_COORD = {
        new Translation2d(14.58,0.47), // 1
        new Translation2d(14.58,1.03), // 2
        new Translation2d(14.58,1.62), // 3
        new Translation2d(14.58,2.15), // 4
        new Translation2d(14.58,2.73), // 5
        new Translation2d(14.58,3.28), // 6
        new Translation2d(14.58, 3.85), // 7
        new Translation2d(14.58,4.41), // 8
        new Translation2d(14.58, 4.95), // 9
    };

    public static final Translation2d[] BLUE_NODE_COORD = {
        new Translation2d(1.94,4.98), // 1
        new Translation2d(1.94,4.39), // 2
        new Translation2d(1.94,3.86), // 3
        new Translation2d(1.94,3.29), // 4
        new Translation2d(1.94,2.72), // 5
        new Translation2d(1.94,2.18), // 6
        new Translation2d(1.94,1.60), // 7
        new Translation2d(1.94,1.06), // 8
        new Translation2d(1.94,0.49), // 9
    };

    public static final Translation2d[] BLUE_SUBSTATIONS = {
        new Translation2d(15.2,7.33), // LEFT DOUBLE SUBSTATION FOR BLUE ALLIANCE SCORING
        new Translation2d(15.2,6.03), //RIGHT DOUBLE SUBSTATION FOR BLUE ALLIANCE SCORING
        new Translation2d(14.308,7.507) // SINGLE SUBSTATION FOR SCORING BLUE ALLIANCE
    };

    public static final Translation2d[] RED_SUBSTATIONS = {
        new Translation2d(1.2038, 6.14), // LEFT DOUBLE SUBSTATION FOR RED ALLIANCE SCORING
        new Translation2d(1.2038, 7.47), // RIGHT DOUBLE SUBSTATION FOR RED ALLIANCE SCORING
        new Translation2d(2.352,7.389) // SINGLE SUBSTATION FOR RED ALLIANCE SCORING 
    };

    public static double getInvertedWidthInches(double blueAllianceWidth) {
        return HALF_FIELD_WIDTH_TIMES_2_INCHES - blueAllianceWidth;
    }

    public static double convertToRedWidthMeters(double blueAllianceWidth) {
        return HALF_FIELD_WIDTH_TIMES_2_METERS - blueAllianceWidth;
    }
    
}
