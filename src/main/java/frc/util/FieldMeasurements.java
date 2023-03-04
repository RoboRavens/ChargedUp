package frc.util;

public class FieldMeasurements {
    // Field measurements in inches.
    // There is a .03 inch discrepancy in the field measurements, likely due to a rounding error on FIRST's part.
    // Half the field as measured by its components, multiplied by two, is .03 inches narrower than the overall field.
    // That comes out to .762 millimeters, so not enough to worry about.
    public static final double OVERALL_FIELD_WIDTH_INCHES = 651.25;
    public static final double HALF_FIELD_WIDTH_INCHES = 325.61;

    // Field elements used as references.
    public static final double DOUBLE_SUBSTATION_WIDTH_INCHES = 27.125;
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

    // Loading zone.
    public static final double LOADING_ZONE_WIDE_AREA_WIDTH_INCHES = DOUBLE_SUBSTATION_WIDTH_INCHES + LOADING_ZONE_WIDE_AREA_WIDTH_PAST_DOUBLE_SUBSTATION_INCHES;
    public static final double LOADING_ZONE_WIDE_AREA_HEIGHT_INCHES = 99.07;
    public static final double LOADING_ZONE_NARROW_AREA_WIDTH_INCHES = 119;
    public static final double LOADING_ZONE_NARROW_AREA_HEIGHT_INCHES = 50.5;

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
    public static double APRILTAG_123_WIDTH_INCHES = (HALF_FIELD_WIDTH_INCHES * 2) - APRILTAG_1_8_HEIGHT_INCHES;
    public static double APRILTAG_1_WIDTH_INCHES = APRILTAG_123_WIDTH_INCHES;
    public static double APRILTAG_2_WIDTH_INCHES = APRILTAG_123_WIDTH_INCHES;
    public static double APRILTAG_3_WIDTH_INCHES = APRILTAG_123_WIDTH_INCHES;
    public static double APRILTAG_4_WIDTH_INCHES = (HALF_FIELD_WIDTH_INCHES * 2) - APRILTAG_4_5_HEIGHT_INCHES;
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

    // Loading zone.
    public static final double LOADING_ZONE_WIDE_AREA_WIDTH_METERS = LOADING_ZONE_WIDE_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_WIDE_AREA_HEIGHT_METERS = LOADING_ZONE_WIDE_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_NARROW_AREA_WIDTH_METERS = LOADING_ZONE_NARROW_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double LOADING_ZONE_NARROW_AREA_HEIGHT_METERS = LOADING_ZONE_NARROW_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;

    // Neutral zone.
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_WIDTH_METERS = NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_METERS = NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_WIDTH_METERS = NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_HEIGHT_METERS = NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_WIDTH_METERS = NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_WIDTH_INCHES * INCHES_TO_METERS_CONVERSION;
    public static final double NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_HEIGHT_METERS = NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_HEIGHT_INCHES * INCHES_TO_METERS_CONVERSION;

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
    
}
