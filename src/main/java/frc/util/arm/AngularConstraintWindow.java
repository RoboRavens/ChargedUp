package frc.util.arm;

public class AngularConstraintWindow {
    private double lowerBound;
    private double upperBound;

    public AngularConstraintWindow(double lowerBound, double upperBound) {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }
    
    public double getLowerBound() {
        return lowerBound;
    }
    
    public double getUpperBound() {
        return upperBound;
    }
}
