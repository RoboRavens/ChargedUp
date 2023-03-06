package frc.util;

public class ArmSetpoint {
    private String name;
    private int extensionSetpoint;
    private int rotationSetpoint;

    public ArmSetpoint(String name, int extensionSetpoint, int rotationSetpoint) {
        this.name = name;
        this.extensionSetpoint = extensionSetpoint;
        this.rotationSetpoint = rotationSetpoint;
    }

    public String getName() {
        return name;
    }
    
    public int getExtensionSetpoint() {
        return extensionSetpoint;
    }

    public int getRotationSetpoint() {
        return rotationSetpoint;
    }
}
