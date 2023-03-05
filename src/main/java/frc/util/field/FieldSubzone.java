package frc.util.field;

import java.awt.Point;

// import java.awt.Point2D;
import java.awt.Rectangle;

import java.awt.geom.Rectangle2D;

public class FieldSubzone {
    private String name;
    private Point southwestCorner;
    private Point northeastCorner;
    private Rectangle2D boundingBox;// = new Rectangle2D.Double();

    private Rectangle test = new Rectangle(.5, .23, .12, .12);

    public FieldSubzone(String name, double southwestCornerX, double southwestCornerY, double width, double height) {
        this.name = name;

        Point southwestCorner = new Point(southwestCornerX, southwestCornerY);
        Point northeastCorner = new Point(southwestCornerX + width, southwestCornerY + height);

        this.southwestCorner = southwestCorner;
        this.northeastCorner = northeastCorner;
        
        this.boundingBox = new Rectangle2D.Double(southwestCornerX, southwestCornerY, width, height);
    }

    public String getName() {
        return name;
    }

    public Rectangle2D getBoundingBox() {
        return boundingBox;
    }

    public Point getSouthwestCorner() {
        return southwestCorner;
    }

    public Point getNortheastCorner() {
        return northeastCorner;
    }
}