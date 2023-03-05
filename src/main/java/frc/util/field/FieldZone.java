package frc.util.field;

import java.util.ArrayList;
//import org.opencv.core.Point;
//import org.opencv.core.Point3;
//import org.opencv.core.Rect;

import java.awt.Rectangle;
import java.awt.Point;

public class FieldZone {
    private String name;
    private ArrayList<FieldSubzone> subzones = new ArrayList<FieldSubzone>();
    private Point southwestCorner;
    private Point northeastCorner;
    private Rectangle boundingBox;
    // private Rectangle rectangle;

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
        southwestCorner.x = Math.min(southwestCorner.x, subzone.getSouthwestCorner().x);
        southwestCorner.y = Math.min(southwestCorner.y, subzone.getSouthwestCorner().y);
        northeastCorner.x = Math.max(northeastCorner.x, subzone.getNortheastCorner().x);
        northeastCorner.y = Math.max(northeastCorner.y, subzone.getNortheastCorner().y);
    }

    public void generateBoundingBox() {
        boundingBox = new Rect(southwestCorner, northeastCorner);
    }

    public String getName() {
        return name;
    }

    public void output() {
        System.out.println("Zone: " + name);
        System.out.println("SW Corner: " + southwestCorner.x + ", " + southwestCorner.y);
        System.out.println("NE Corner: " + northeastCorner.x + ", " + northeastCorner.y);
        System.out.println(" Bounding box: x: " + boundingBox.x + " y: " + boundingBox.y + " width: " + boundingBox.width + " height: " + boundingBox.height);
    }
}
