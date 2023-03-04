package frc.util;

import org.opencv.core.Point;
import org.opencv.core.Rect;

public class FieldSubzone {
    private String name;
    private Point southwestCorner;
    private Point northeastCorner;
    private Rect boundingBox = new Rect();

    public FieldSubzone(String name, double southwestCornerX, double southwestCornerY, double northeastCornerX, double northeastCornerY) {
        this.name = name;

        Point southwestCorner = new Point(southwestCornerX, southwestCornerY);
        Point northeastCorner = new Point(northeastCornerX, northeastCornerY);

        this.southwestCorner = southwestCorner;
        this.northeastCorner = northeastCorner;
        
        this.boundingBox = new Rect(southwestCorner, northeastCorner);
    }

    public String getName() {
        return name;
    }

    public Rect getBoundingBox() {
        return boundingBox;
    }

    public Point getSouthwestCorner() {
        return southwestCorner;
    }

    public Point getNortheastCorner() {
        return northeastCorner;
    }
}