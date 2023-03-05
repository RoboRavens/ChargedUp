package frc.util.field;

public class MirroredSubzone {
    private FieldSubzone blueSubzone;
    private FieldSubzone redSubzone;

    public MirroredSubzone(String name, double southwestCornerX, double southwestCornerY, double width, double height) {
        blueSubzone = new FieldSubzone(name, southwestCornerX, southwestCornerY, width, height);
        redSubzone = new FieldSubzone(name, FieldMeasurements.convertToRedWidthMeters(southwestCornerX), southwestCornerY, width, height);
    }

    public FieldSubzone getBlueSubzone() {
        return blueSubzone;
    }

    public FieldSubzone getRedSubzone() {
        return redSubzone;
    }
}
