package frc.util;

import java.util.ArrayList;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public class FieldZone {
    private ArrayList<FieldSubzone> subzones = new ArrayList<FieldSubzone>();
    private Point southwestCorner;
    private Point northeastCorner;
    private Rect boundingBox;

    // Createa field zone, which must be initialized with at least one subzone.
    public FieldZone(FieldSubzone initialSubzone) {
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
}
