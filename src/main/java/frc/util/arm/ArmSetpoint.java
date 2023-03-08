package frc.util.arm;

import frc.robot.Constants;

public class ArmSetpoint {
    private String name;
    private double extensionSetpoint;
    private double rotationSetpoint;

    public ArmSetpoint(String name, double extensionSetpoint, double rotationSetpoint) {
        this.name = name;
        this.extensionSetpoint = extensionSetpoint;
        this.rotationSetpoint = rotationSetpoint;
    }

    public String getName() {
        return name;
    }
    
    public double getExtensionSetpoint() {
        return extensionSetpoint;
    }

    public double getRotationSetpoint() {
        return rotationSetpoint;
    }

    public double getRotationSetpointDegrees() {
        return rotationSetpoint / Constants.ARM_DEGREES_TO_ENCODER_UNITS;
    }
}
