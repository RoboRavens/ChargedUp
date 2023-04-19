package frc.util;

public class Color {
    private String name;
    private int red;
    private int green;
    private int blue;
    
    public Color(String name, int red, int green, int blue) {
        this.name = name;
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public String getName() {
        return name;
    }

    public int getRed() {
        return red;
    }

    public int getGreen() {
        return green;
    }

    public int getBlue() {
        return blue;
    }
}
