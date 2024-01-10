package frc.robot;
import edu.wpi.first.wpilibj.XboxController;

public final class ButtonMapping {
    public final static int RaiseLinearActuator = XboxController.Button.kX.value;
    public final static int LowerLinearActuator = XboxController.Button.kB.value;
    public final static int DumpUp = 0; // D-Pad Up
    public final static int DumpDown = 180; // D-Pad Down
    public final static int OperatorDrive = 90; // D-Pad Right
    public final static int DigForward = XboxController.Button.kY.value;
    public final static int DigReverse = XboxController.Button.kA.value;
    public final static int RetractLeadscrew = XboxController.Button.kRightBumper.value;
    public final static int ExtendLeadscrew = XboxController.Button.kLeftBumper.value;
}
