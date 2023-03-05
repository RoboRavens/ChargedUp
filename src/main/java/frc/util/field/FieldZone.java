package frc.util.field;

import java.util.ArrayList;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D;

public class FieldZone {
    private String name;
    private ArrayList<FieldSubzone> subzones = new ArrayList<FieldSubzone>();
    private Point2D southwestCorner;
    private Point2D northeastCorner;
    private Rectangle2D boundingBox;

    // Create a field zone, which must be initialized with at least one subzone.
    public FieldZone(String name, FieldSubzone initialSubzone) {
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

    public String getName() {
        return name;
    }

    public void output() {
        System.out.println("Zone: " + name);
        System.out.println("SW Corner: " + southwestCorner.getX() + ", " + southwestCorner.getY());
        System.out.println("NE Corner: " + northeastCorner.getX() + ", " + northeastCorner.getY());
        
        System.out.println(" Bounding box: x: " + boundingBox.getX() + " y: " + boundingBox.getY() + " width: " + boundingBox.getWidth() + " height: " + boundingBox.getHeight());
        System.out.println();
        System.out.println();
    }

    public void outputSubzones() {
        System.out.println("Zone: " + name);
        System.out.println("SW Corner: " + southwestCorner.getX() + ", " + southwestCorner.getY());
        System.out.println("NE Corner: " + northeastCorner.getX() + ", " + northeastCorner.getY());
        System.out.println();
        // System.out.println(" Bounding box: x: " + boundingBox.getX() + " y: " + boundingBox.getY() + " width: " + boundingBox.getWidth() + " height: " + boundingBox.getHeight());

        for (FieldSubzone subzone : subzones) {
            subzone.output();
        }

        System.out.println();
        System.out.println();
    }
}
