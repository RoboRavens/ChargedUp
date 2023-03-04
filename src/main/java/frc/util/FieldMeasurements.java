package frc.util;

public class FieldMeasurements {
    // Field measurements in inches.
    // There is a .03 inch discrepancy in the field measurements, likely due to a rounding error on FIRST's part.
    // Half the field as measured by its components, multiplied by two, is .03 inches narrower than the overall field.
    // That comes out to .762 millimeters, so not enough to worry about.
    public static final double OVERALL_FIELD_WIDTH_INCHES = 651.25;
    public static final double HALF_FIELD_WIDTH_INCHES = 325.61;
    
    // Community.
    public static final double DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES = 54.25;
    public static final double COMMUNITY_LENGTH_INCHES = 216.5;
    public static final double COMMUNITY_NORTHERN_SECTION_WIDTH_INCHES = 78; // 132.25 - 54.25.
    public static final double COMMUNITY_NORTHERN_SECTION_HEIGHT_INCHES = 59.39;
    public static final double COMMUNITY_COOP_SECTION_WIDTH_INCHES = 60.69;
    public static final double COMMUNITY_COOP_SECTION_HEIGHT_INCHES = 96; // 8 feet, same as the charge station.
    public static final double COMMUNITY_SOUTHERN_SECTION_WIDTH_INCHES = 138.87; // 224 minues 85.13, from the layout guide.
    public static final double COMMUNITY_SOUTHERN_SECTION_HEIGHT_INCHES = 59.39;

    // Charge station, which is also part of the community.
    public static final double COMMUNITY_CHARGE_STATION_WIDTH_INCHES = 76.125; // 6 foot 4 and a quarter.
    public static final double COMMUNITY_CHARGE_STATION_HEIGHT_INCHES = 96; // 8 feet, same as the charge station.

    // Loading zone.
    public static final double LOADING_ZONE_WIDE_AREA_WIDTH_INCHES = 145.25;
    public static final double LOADING_ZONE_WIDE_AREA_HEIGHT_INCHES = 99.07;
    public static final double LOADING_ZONE_NARROW_AREA_WIDTH_INCHES = 119;
    public static final double LOADING_ZONE_NARROW_AREA_HEIGHT_INCHES = 50.5;

    // Neutral zone.
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_WIDTH_INCHES = 61.36;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_INCHES = 50.5;
    public static final double NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_WIDTH_INCHES = HALF_FIELD_WIDTH_INCHES - COMMUNITY_NORTHERN_SECTION_WIDTH_INCHES - DISTANCE_FROM_WALL_TO_COMMUNITY_INCHES;

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
    
    
}
