package frc.robot.subsystems.Dashboards;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PIDTuner {
    private static final NetworkTable ReactDash = NetworkTableInstance.getDefault().getTable("ReactDash");
    private final NetworkTable mainTable = PIDTuner.ReactDash.getSubTable("Main");
    private DoubleSubscriber _pSub;
    private DoubleSubscriber _iSub;
    private DoubleSubscriber _dSub;

    public PIDTuner() {
        _pSub = mainTable.getDoubleTopic("dpub/pInput").subscribe(0);
        _iSub = mainTable.getDoubleTopic("dpub/iInput").subscribe(0);
        _dSub = mainTable.getDoubleTopic("dpub/dInput").subscribe(0);
    }

    public double getP() {
        return _pSub.get();
    }

    public double getI() {
        return _iSub.get();
    }

    public double getD() {
        return _dSub.get();
    }
}
