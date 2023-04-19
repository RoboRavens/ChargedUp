package frc.util.field;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.field.FieldZones.FieldMacroZone;

import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D;

public class FieldZone {
    private String name;
    private ArrayList<FieldSubzone> subzones = new ArrayList<FieldSubzone>();
    private Point2D southwestCorner;
    private Point2D northeastCorner;
    private Rectangle2D boundingBox;
    private FieldMacroZone macroZone;
    private Alliance zoneOwner;

    // Create a field zone, which must be initialized with at least one subzone.
    public FieldZone(Alliance owner, FieldMacroZone macroZone, String name, FieldSubzone initialSubzone) {
        this.zoneOwner = owner;
        this.macroZone = macroZone;
        this.name = name;
        subzones.add(initialSubzone);
        this.southwestCorner = initialSubzone.getSouthwestCorner();
        this.northeastCorner = initialSubzone.getNortheastCorner();
    }

    public void addSubzone(FieldSubzone subzone) {
        this.subzones.add(subzone);

        // Update the bounding box's boundaries to any boundary that is more extreme in the subzone,
        // than in the current field zone.
        double newSouthwestCornerX = Math.min(southwestCorner.getX(), subzone.getSouthwestCorner().getX());
        double newSouthwestCornerY = Math.min(southwestCorner.getY(), subzone.getSouthwestCorner().getY());
        southwestCorner = new Point2D.Double(newSouthwestCornerX, newSouthwestCornerY);

        double newNortheastCornerX = Math.max(northeastCorner.getX(), subzone.getNortheastCorner().getX());
        double newNortheastCornerY = Math.max(northeastCorner.getY(), subzone.getNortheastCorner().getY());
        northeastCorner = new Point2D.Double(newNortheastCornerX, newNortheastCornerY);
    }

    public void generateBoundingBox() {
        double width = northeastCorner.getX() - southwestCorner.getX();
        double height = northeastCorner.getY() - southwestCorner.getY();

        boundingBox = new Rectangle2D.Double(southwestCorner.getX(), southwestCorner.getY(), width, height);
    }

    public boolean containsPoint(Point2D point) {
        return boundingBox.contains(point);
    }

    public FieldSubzone subzonesContainPoint(Point2D point) {
        Optional<FieldSubzone> result =
            subzones.stream().filter(subzone -> subzone.containsPoint(point) ).findFirst();
        
           // System.out.println();
        
        if (result.isPresent()) {
            return result.get();
        }
        else {
            return null;
        }
    }

    public String getName() {
        return name;
    }

    public FieldMacroZone getMacroZone() {
        return macroZone;
    }

    public Alliance getZoneOwner() {
        return zoneOwner;
    }

    public void output() {
      /*   System.out.println("Zone: " + name);
        System.out.println("SW Corner: " + southwestCorner.getX() + ", " + southwestCorner.getY());
        System.out.println("NE Corner: " + northeastCorner.getX() + ", " + northeastCorner.getY());
        
        System.out.println();
        System.out.println(); */
    }

    public void outputSubzones() {
       /*  System.out.println("Zone: " + name);
        System.out.println("SW Corner: " + southwestCorner.getX() + ", " + southwestCorner.getY());
        System.out.println("NE Corner: " + northeastCorner.getX() + ", " + northeastCorner.getY());
        System.out.println(); */

        for (FieldSubzone subzone : subzones) {
            subzone.output();
        }

       // System.out.println();
       // System.out.println();
    }
}
