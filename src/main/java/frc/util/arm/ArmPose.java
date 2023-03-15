// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ArmPose {
    private double armAngleDegrees;
    private double armAngleRadians;
    private double armLengthToHitConstraintNativeUnits;
    private double armRotationMinimumBoundNativeUnits;
    private double armRotationMaximumBoundNativeUnits;

    // We can either remove these or uncomment the getters
    // if we want to set them and use them for diagnostic data.
    private double armRotationMinimumBoundDegrees;
    private double armRotationMaximumBoundDegrees;
    private double armNetLengthInches;
    private double armNetHeightInches;
    private double armNetExtensionInches;
    private double armNetLengthNativeUnits;
    private double armNetHeightNativeUnits;
    private double armNetExtensionNativeUnits;
    private double armLengthToHitConstraintInches;
    private double heightAtMaxExtensionInches;
    private double widthAtMaxExtensionInches;

    // Construct the ArmPose based on the starting values for the arm.
    public ArmPose(double startingDegress) {
        this.setArmAngleDegrees(startingDegress);
    }

    public void calculateInstantaneousMaximums() {
        calculateInstantaneousMaxRotation();
        calculateInstantaneousMaxExtension();
    }

    public void calculateInstantaneousMaxRotation() {
        AngularConstraintWindow window = getMaxRotationAngularConstraintWindow(armNetExtensionInches, armAngleDegrees);

        this.armRotationMinimumBoundDegrees = window.getLowerBound();
        this.armRotationMinimumBoundNativeUnits = window.getLowerBound() * Constants.ARM_DEGREES_TO_ENCODER_UNITS;
        this.armRotationMaximumBoundDegrees = window.getUpperBound();
        this.armRotationMaximumBoundNativeUnits = window.getUpperBound() * Constants.ARM_DEGREES_TO_ENCODER_UNITS;

    }

    public static AngularConstraintWindow getMaxRotationAngularConstraintWindow(double armExtensionInches, double angleDegrees) {
        double armLengthInches = Constants.ARM_BASE_LENGTH_INCHES + armExtensionInches;

        // Upper quadrants height constraint angle.
        double remainingVerticalInches = Constants.VERTICAL_EXPANSION_ROOM - Constants.FULCRUM_HEIGHT_INCHES;
        double sinOfHeightConstraintAngle = remainingVerticalInches / armLengthInches;
        double heightConstraintAngle = Math.asin(sinOfHeightConstraintAngle);
        double heightConstraintAngleDegrees = Math.toDegrees(heightConstraintAngle);

        // Lower quadrants height constraint angle.
        double remainingNegativeVerticalInches = Constants.FULCRUM_HEIGHT_INCHES - Constants.SAFETY_MARGIN_INCHES;
        double sinOfNegativeHeightConstraintAngle = remainingNegativeVerticalInches / armLengthInches;
        double lowerQuadrantsHeightConstraintAngle = Math.asin(sinOfNegativeHeightConstraintAngle);
        double negativeHeightConstraintAngleDegrees = Math.toDegrees(lowerQuadrantsHeightConstraintAngle);

        // Width constraint angle.
        double cosOfWidthConstraintAngle = Constants.HORIZONTAL_EXPANSION_ROOM / armLengthInches;
        double widthConstraintAngle = Math.acos(cosOfWidthConstraintAngle);
        double widthConstraintAngleDegrees = Math.toDegrees(widthConstraintAngle);


        if (Double.isNaN(heightConstraintAngleDegrees)) {
            heightConstraintAngleDegrees = 90;
        }

        if (Double.isNaN(negativeHeightConstraintAngleDegrees)) {
            negativeHeightConstraintAngleDegrees = 90;
        }

        if (Double.isNaN(widthConstraintAngleDegrees)) {
            widthConstraintAngleDegrees = 0;
        }
        

        // Lower quadrants width constraint angle.
        double lowerQuadrantsWidthConstraintAngleDegrees = widthConstraintAngleDegrees + 90;


        // Translate the raw angles to our coordinate system.
        heightConstraintAngleDegrees = 90 - heightConstraintAngleDegrees;
        negativeHeightConstraintAngleDegrees += 90;
        widthConstraintAngleDegrees = 90 - widthConstraintAngleDegrees;


        // Special cases, if the arm is EXACTLY at a quadrant boundary.
        // These aren't technically mathematically correct,
        // but it doesn't matter because these boundaries
        // will be erased as soon as the arm moves a single encoder tick
        // in either direction.
        if (angleDegrees == 0) {
            // Arm is straight up.
            heightConstraintAngleDegrees = widthConstraintAngleDegrees * -1;
        }

        if (angleDegrees == 90) {
            // Arm is straight forward.
            widthConstraintAngleDegrees = negativeHeightConstraintAngleDegrees;
        }

        if (angleDegrees == -90) {
            // Arm is straight backward.
            widthConstraintAngleDegrees = negativeHeightConstraintAngleDegrees;
        }

        // Once we have all the constraints, create a window based on which quadrant the arm is actually in.
        // Constraints in other quadrants are not relevant.
        double finalWidthConstraintAngleDegrees = widthConstraintAngleDegrees;
        double finalHeightConstraintAngleDegrees = heightConstraintAngleDegrees;

        // Check if the arm is in a lower quadrant and if it is, use the lower quadrant limits.
        if (Math.abs(angleDegrees) > 90) {
            finalWidthConstraintAngleDegrees = lowerQuadrantsWidthConstraintAngleDegrees;
            finalHeightConstraintAngleDegrees = negativeHeightConstraintAngleDegrees;
        }

        // If the arm is "backwards", just multiple by -1 since in our coordinate system the Y axis is 0.
        if (angleDegrees < 0) {
            finalWidthConstraintAngleDegrees  *= -1;
            finalHeightConstraintAngleDegrees  *= -1;
        }

        // Last step is to return the window.
        AngularConstraintWindow window = new AngularConstraintWindow(Math.min(finalWidthConstraintAngleDegrees, finalHeightConstraintAngleDegrees), Math.max(finalWidthConstraintAngleDegrees, finalHeightConstraintAngleDegrees));
        SmartDashboard.putNumber("WindowMin", window.getLowerBound());
        SmartDashboard.putNumber("WindowMax", window.getUpperBound());

        
        SmartDashboard.putNumber("WindowDegress", angleDegrees);
        SmartDashboard.putNumber("LQWCAD", lowerQuadrantsWidthConstraintAngleDegrees);
        SmartDashboard.putNumber("HCAD", heightConstraintAngleDegrees);
        SmartDashboard.putNumber("NHCAD", negativeHeightConstraintAngleDegrees);
        SmartDashboard.putNumber("WCAD", widthConstraintAngleDegrees);
        /* 
        if (Double.isNaN(this.armRotationMinimumBoundDegrees)) {
            this.armRotationMinimumBoundDegrees = -1 * Constants.ARM_MAX_ROTATION_DEGREES;
            this.armRotationMinimumBoundDegrees = -1 * Constants.ARM_MAX_ROTATION_DEGREES * Constants.ARM_DEGREES_TO_ENCODER_UNITS;
        }

        if (Double.isNaN(this.armRotationMaximumBoundDegrees)) {
            this.armRotationMaximumBoundDegrees = Constants.ARM_MAX_ROTATION_DEGREES;
            this.armRotationMaximumBoundDegrees = Constants.ARM_MAX_ROTATION_DEGREES * Constants.ARM_DEGREES_TO_ENCODER_UNITS;
        }
        */


        return window;
    }



    public void calculateInstantaneousMaxExtension() {
        ArmMaximumConstraint maxConstraintAtAngle = calculateMaxExtensionAtAngleDegrees(armAngleDegrees);

        this.armLengthToHitConstraintInches = maxConstraintAtAngle.getArmLengthToHitConstraintInches();
        this.armLengthToHitConstraintNativeUnits = maxConstraintAtAngle.getArmLengthToHitConstraintNativeUnits();
        this.heightAtMaxExtensionInches = maxConstraintAtAngle.getHeightAtMaxExtensionInches();
        this.widthAtMaxExtensionInches = maxConstraintAtAngle.getWidthAtMaxExtensionInches();
    }

    public static ArmMaximumConstraint calculateMaxExtensionAtAngleDegrees(double armAngleDegrees) {
        double armAngleRadians = Math.toRadians(armAngleDegrees);
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

        // If the arm is "behind" the fulcrum, we'll need to multiply the position by -1 later.
        double armWidthAdditionCoefficient = 1;
        if (armAngleDegrees < 0) {
            armWidthAdditionCoefficient = -1;
        }

        double armLengthToHitConstraintInches = armLengthToHitConstraint;
        double armLengthToHitConstraintNativeUnits = armLengthToHitConstraint / Constants.ARM_EXTENSION_PER_UNIT;

        // For output or debugging, calculate the coordinates that the arm would be at its maximum extension.
        double heightAtMaxExtensionInches = armBaseNetVerticalAtAngle + armLengthToHitConstraint * cosAngle;
        double widthAtMaxExtensionInches = (armBaseHorizontalAtAngle + armLengthToHitConstraint * sinAngle) * armWidthAdditionCoefficient;

       return new ArmMaximumConstraint(armAngleDegrees, armAngleRadians, armLengthToHitConstraintNativeUnits, armLengthToHitConstraintInches, heightAtMaxExtensionInches, widthAtMaxExtensionInches);
    }

    public double getArmAngleRadians() {
        return armAngleRadians;
    }

    public double getArmRotationNativeUnits() {
        return armAngleDegrees * Constants.ARM_DEGREES_TO_ENCODER_UNITS;

    }

    public void setArmAngleRadians(double armAngleRadians) {
        this.armAngleRadians = armAngleRadians;
        this.armAngleDegrees = Math.toDegrees(armAngleRadians);
    }

    public void setArmAngleDegrees(double armAngleDegrees) {
        this.armAngleDegrees = armAngleDegrees;
        this.armAngleRadians = Math.toRadians(armAngleDegrees);
    }

    public double getArmRotationMinimumBoundNativeUnits() {
        return armRotationMinimumBoundNativeUnits;
    }

    public double getArmRotationMaximumBoundNativeUnits() {
        return armRotationMaximumBoundNativeUnits;
    }
/*
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

        public double getHeightAtMaxExtensionInches() {
        return heightAtMaxExtensionInches;
    }

    public double getWidthAtMaxExtensionInches() {
        return widthAtMaxExtensionInches;
    }

    public double getArmLengthToHitConstraintInches() {
        return armLengthToHitConstraintInches;
    }
    */

    public int getArmLengthToHitConstraintNativeUnits() {
        return (int) Math.round(armLengthToHitConstraintNativeUnits);
    }
    

}
