// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import frc.robot.Constants;

public class ArmPose {
    private double armAngleDegrees;
    private double armAngleRadians;
    private double armNetLengthInches;
    private double armNetHeightInches;
    private double armNetExtensionInches;
    private double armNetLengthNativeUnits;
    private double armNetHeightNativeUnits;
    private double armNetExtensionNativeUnits;
    private double armLengthToHitConstraintInches;
    private double armLengthToHitConstraintNativeUnits;
    private double heightAtMaxExtensionInches;
    private double widthAtMaxExtensionInches;

    // Construct the ArmPose based on the starting values for the arm.
    public ArmPose() {
        this.setArmAngleDegrees(Constants.ARM_STARTING_DEGREES);
    }

    public void calculateInstantaneousMaximums() {
        calculateInstaneousMaxExtension();
    }

    public void calculateInstaneousMaxExtension() {
        armAngleRadians = Math.toRadians(armAngleDegrees);
        double sinAngle = Math.abs(Math.sin(armAngleRadians));
        double cosAngle = Math.abs(Math.cos(armAngleRadians));

        // Calculate the arm's location without any extension using the base length of the arm.
        double armBaseVerticalAtAngle = Constants.ARM_BASE_LENGTH_INCHES * cosAngle;
        double armBaseNetVerticalAtAngle = armBaseVerticalAtAngle + Constants.FULCRUM_HEIGHT_INCHES;
        double armBaseHorizontalAtAngle = Constants.ARM_BASE_LENGTH_INCHES * sinAngle;

        // Calculate how much room there is to expand at the given angle.
        double remainingVerticalInchesAtAngle = Constants.VERTICAL_EXPANSION_ROOM - armBaseNetVerticalAtAngle;
        double remainingHorizontalInchesAtAngle = Constants.HORIZONTAL_EXPANSION_ROOM - armBaseHorizontalAtAngle;

        // Calculate the arm lengths that would hit the horizontal and vertical limits.
        // If the arm is at an increment of 90, divide-by-0 errors are possible.
        // In these circumstances the arm could extend infinitely in that direction, so just
        // set it the maximum (so the other axis will be the limit) by default
        // and then trim it when the sin/cos is NOT 0.
        double armLengthToHitMaxVertical = Constants.VERTICAL_EXPANSION_ROOM;
        double armLengthToHitMaxHorizontal = Constants.HORIZONTAL_EXPANSION_ROOM;

        if (cosAngle != 0) {
            armLengthToHitMaxVertical = remainingVerticalInchesAtAngle / cosAngle;
        }

        if (sinAngle != 0) {
            armLengthToHitMaxHorizontal = remainingHorizontalInchesAtAngle / sinAngle;
        }

        // Take the smaller of the two max lengths in order to comply with both constraints.
        double armLengthToHitConstraint = Math.min(armLengthToHitMaxVertical, armLengthToHitMaxHorizontal);

        // If we're in a lower quadrant, just set it to zero because ground intake and scoring are at zero.
        // No need to get fancy in the lower quadrant.
        if (Math.abs(armAngleDegrees) >= 90) {
            armLengthToHitConstraint = 0;

            // Recalculate the arm height given that we should subtract from the fulcrum instead of adding to it.
            armBaseNetVerticalAtAngle = Constants.FULCRUM_HEIGHT_INCHES - armBaseVerticalAtAngle;
        }

        // If the arm is "behind" the behind, we'll need to multiply the position by -1 later.
        double armWidthAdditionCoefficient = 1;
        if (armAngleDegrees < 0) {
            armWidthAdditionCoefficient = -1;
        }

        // Update the class fields.
        this.armLengthToHitConstraintInches = armLengthToHitConstraint;
        this.armLengthToHitConstraintNativeUnits = armLengthToHitConstraint / Constants.ARM_EXTENSION_PER_UNIT;

        // For output or debugging, calculate the coordinates that the arm would be at its maximum extension.
        this.heightAtMaxExtensionInches = armBaseNetVerticalAtAngle + armLengthToHitConstraint * cosAngle;
        this.widthAtMaxExtensionInches = (armBaseHorizontalAtAngle + armLengthToHitConstraint * sinAngle) * armWidthAdditionCoefficient;
    }

    public double getArmAngleRadians() {
        return armAngleRadians;
    }

    public void setArmAngleRadians(double armAngleRadians) {
        this.armAngleRadians = armAngleRadians;
        this.armAngleDegrees = Math.toDegrees(armAngleRadians);
    }

    public void setArmAngleDegrees(double armAngleDegrees) {
        this.armAngleDegrees = armAngleDegrees;
        this.armAngleRadians = Math.toRadians(armAngleDegrees);
    }

    public double getArmNetLengthInches() {
        return armNetLengthInches;
    }

    public double getArmNetHeightInches() {
        return armNetHeightInches;
    }

    public double getArmNetExtensionInches() {
        return armNetExtensionInches;
    }

    public double getArmAngleDegrees() {
        return armAngleDegrees;
    }

    public double getArmNetLengthNativeUnits() {
        return armNetLengthNativeUnits;
    }

    public double getArmNetHeightNativeUnits() {
        return armNetHeightNativeUnits;
    }

    public double getArmNetExtensionNativeUnits() {
        return armNetExtensionNativeUnits;
    }

    public int getArmLengthToHitConstraintNativeUnits() {
        return (int) Math.round(armLengthToHitConstraintNativeUnits);
    }
    
    public double getHeightAtMaxExtensionInches() {
        return heightAtMaxExtensionInches;
    }

    public double getWidthAtMaxExtensionInches() {
        return widthAtMaxExtensionInches;
    }

    public double getArmLengthToHitConstraintInches() {
        return armLengthToHitConstraintInches;
    }
}
