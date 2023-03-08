package frc.util;

public class ArmMaximumConstraint {
    private double armAngleDegrees;
    private double armAngleRadians;
    private double armLengthToHitConstraintNativeUnits;
    private double armLengthToHitConstraintInches;
    private double heightAtMaxExtensionInches;
    private double widthAtMaxExtensionInches;

    public ArmMaximumConstraint(double armAngleDegrees, double armAngleRadians,
        double armLengthToHitConstraintNativeUnits, double armLengthToHitConstraintInches,
        double heightAtMaxExtensionInches, double widthAtMaxExtensionInches) {
    this.armAngleDegrees = armAngleDegrees;
    this.armAngleRadians = armAngleRadians;
    this.armLengthToHitConstraintNativeUnits = armLengthToHitConstraintNativeUnits;
    this.armLengthToHitConstraintInches = armLengthToHitConstraintInches;
    this.heightAtMaxExtensionInches = heightAtMaxExtensionInches;
    this.widthAtMaxExtensionInches = widthAtMaxExtensionInches;
    }

    public double getArmAngleDegrees() {
        return armAngleDegrees;
    }

    public double getArmAngleRadians() {
        return armAngleRadians;
    }

    public double getArmLengthToHitConstraintNativeUnits() {
        return armLengthToHitConstraintNativeUnits;
    }

    public double getArmLengthToHitConstraintInches() {
        return armLengthToHitConstraintInches;
    }

    public double getHeightAtMaxExtensionInches() {
        return heightAtMaxExtensionInches;
    }
    
    public double getWidthAtMaxExtensionInches() {
        return widthAtMaxExtensionInches;
    }
}
