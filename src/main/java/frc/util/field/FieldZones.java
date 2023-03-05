package frc.util.field;

import java.util.ArrayList;
import java.util.Optional;
import java.awt.geom.Point2D;

public class FieldZones {
    private ArrayList<FieldZone> fieldZones = new ArrayList<FieldZone>();
    private FieldZone blueCommunity;
    private FieldZone redCommunity;
    private FieldZone blueLoadingZone;
    private FieldZone redLoadingZone;
    private FieldZone blueChargeStation;
    private FieldZone redChargeStation;
    private FieldZone neutralZone;
    
    // We'll include a none state in case odometry ever breaks by enough that no zone is returned.
    private FieldZone noneZone;

    // Community.
    public static final MirroredSubzone COMMUNITY_NORTHERN_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Community Northern Section",
        FieldMeasurements.COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.COMMUNITY_NORTHERN_SECTION_WIDTH_METERS,
        FieldMeasurements.COMMUNITY_NORTHERN_SECTION_HEIGHT_METERS,
        false);
    
    public static final MirroredSubzone COMMUNITY_COOP_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Community Coop Section",
        FieldMeasurements.COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.COMMUNITY_COOP_SECTION_WIDTH_METERS,
        FieldMeasurements.COMMUNITY_COOP_SECTION_HEIGHT_METERS,
        false);

    public static final MirroredSubzone COMMUNITY_SOUTHERN_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Community Southern Section",
        FieldMeasurements.COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.COMMUNITY_SOUTHERN_SECTION_WIDTH_METERS,
        FieldMeasurements.COMMUNITY_SOUTHERN_SECTION_HEIGHT_METERS,
        false);

    public static final MirroredSubzone COMMUNITY_CHARGE_STATION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Community Charge Station",
        FieldMeasurements.COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.COMMUNITY_CHARGE_STATION_WIDTH_METERS,
        FieldMeasurements.COMMUNITY_CHARGE_STATION_HEIGHT_METERS,
        false);
    
    // Loading zone.
    public static final MirroredSubzone LOADING_ZONE_WIDE_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Loading Zone Wide Section",
        FieldMeasurements.LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.LOADING_ZONE_WIDE_AREA_WIDTH_METERS,
        FieldMeasurements.LOADING_ZONE_WIDE_AREA_HEIGHT_METERS,
        true);

    public static final MirroredSubzone LOADING_ZONE_NARROW_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Loading Zone Narrow Section",
        FieldMeasurements.LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.LOADING_ZONE_NARROW_AREA_WIDTH_METERS,
        FieldMeasurements.LOADING_ZONE_NARROW_AREA_HEIGHT_METERS,
        true);
    
    // Neutral zone. Unlike the other zones, the NZ is NOT mirrored - it's just one large FieldZone.
    // However the width coordinates are defined by halves so need to be doubled.
    public static final FieldSubzone neutralZoneNarrowArea = new FieldSubzone(
        "Neutral Zone Narrow Area",
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_WIDTH_METERS * 2,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_METERS);
    
    public static final FieldSubzone neutralZoneNorthernArea = new FieldSubzone(
        "Neutral Zone Northern Area",
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_WIDTH_METERS * 2,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_HEIGHT_METERS);  
        
    public static final FieldSubzone neutralZoneSouthernArea = new FieldSubzone(
        "Neutral Zone Southern Area",
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_WIDTH_METERS * 2,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_HEIGHT_METERS);
    
    public static final FieldSubzone noneSubzone = new FieldSubzone(
        "None",
        -1,
        -1,
        0,
        0);

    
    
    public FieldZones() {
        // Communities.
        MirroredFieldZone communityMirroredFieldZone = new MirroredFieldZone("Community", COMMUNITY_NORTHERN_SECTION_MIRRORED_SUBZONE);
            communityMirroredFieldZone.addMirroredSubzone(COMMUNITY_COOP_SECTION_MIRRORED_SUBZONE);
            communityMirroredFieldZone.addMirroredSubzone(COMMUNITY_SOUTHERN_SECTION_MIRRORED_SUBZONE);
            communityMirroredFieldZone.addMirroredSubzone(COMMUNITY_CHARGE_STATION_MIRRORED_SUBZONE);

        blueCommunity = communityMirroredFieldZone.getBlueFieldZone();
        redCommunity = communityMirroredFieldZone.getRedFieldZone();
        blueCommunity.generateBoundingBox();
        redCommunity.generateBoundingBox();
        fieldZones.add(blueCommunity);
        fieldZones.add(redCommunity);

        // Loading zones.
        MirroredFieldZone loadingZoneMirroredFieldZone = new MirroredFieldZone("Loading Zone", LOADING_ZONE_WIDE_SECTION_MIRRORED_SUBZONE);
            loadingZoneMirroredFieldZone.addMirroredSubzone(LOADING_ZONE_NARROW_SECTION_MIRRORED_SUBZONE);

        blueLoadingZone = loadingZoneMirroredFieldZone.getBlueFieldZone();
        redLoadingZone = loadingZoneMirroredFieldZone.getRedFieldZone();
        blueLoadingZone.generateBoundingBox();
        redLoadingZone.generateBoundingBox();
        fieldZones.add(blueLoadingZone);
        fieldZones.add(redLoadingZone);

        // Neutral zone.
        neutralZone = new FieldZone("Neutral Zone", neutralZoneNarrowArea);
        neutralZone.addSubzone(neutralZoneNorthernArea);
        neutralZone.addSubzone(neutralZoneSouthernArea);
        neutralZone.generateBoundingBox();
        neutralZoneNarrowArea.setFieldZone(neutralZone);
        neutralZoneNorthernArea.setFieldZone(neutralZone);
        neutralZoneSouthernArea.setFieldZone(neutralZone);

        fieldZones.add(neutralZone);

        // The charge stations are a subzone of their respective communities, but also their own zones.
        MirroredFieldZone chargeStationMirroredFieldZone = new MirroredFieldZone("Charge Station", COMMUNITY_CHARGE_STATION_MIRRORED_SUBZONE);
        blueChargeStation = chargeStationMirroredFieldZone.getBlueFieldZone();
        redChargeStation = chargeStationMirroredFieldZone.getRedFieldZone();
        blueChargeStation.generateBoundingBox();
        redChargeStation.generateBoundingBox();
        fieldZones.add(blueChargeStation);
        fieldZones.add(redChargeStation);

        // Create the none zone.
        noneZone = new FieldZone("None", noneSubzone);
        noneZone.generateBoundingBox();
        noneSubzone.setFieldZone(noneZone);
        fieldZones.add(noneZone);
    }

    public FieldSubzone getPointFieldZone(Point2D point) {
        // If odometry is working correctly we'll always find a zone, but if it's not, we need a default.
        FieldSubzone robotFieldSubzone = noneSubzone;

        // Check each zone to see if the point is within the overall bounding box.
        // If it is, check the subzones to see if its in one of those bounding boxes.
        // If it is, that's the zone/subzone that the robot is centered in.
        // If it is not, hop back out and continue checking the remaining zones.
        for (int i = 0; i < fieldZones.size(); i++) {
            if (fieldZones.get(i).containsPoint(point)) {
                robotFieldSubzone = fieldZones.get(i).subzonesContainPoint(point);

                if (robotFieldSubzone != null) {
                    break;
                }
            }
        }

        return robotFieldSubzone;

//        Optional<FieldZone> result =
//            fieldZones.stream().filter(fieldZone -> fieldZone.containsPoint(point) ).findFirst();
    }

    public void output() {
        System.out.println("Zones:");

        for (FieldZone zone : fieldZones) {
            zone.outputSubzones();
        }

        System.out.println();
        System.out.println();
    }

    public FieldZone getNoneZone() {
        return noneZone;
    }
    
}
