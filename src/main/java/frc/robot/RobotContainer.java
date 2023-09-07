package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import static frc.robot.Constants.ControllerConstants.*;

public class RobotContainer {
    // Subsystems
    private final Drivebase s_Drivebase = new DriveBase();

    // Input Devices
    private final XboxController i_driverXbox = new XboxController(XBOX_CONTROLLER_DRIVER);

    // Commands
    private final XboxDrive c_XboxDrive = new XboxDrive(s_Drivebase, i_driverXbox);

    public RobotContainer() {
        s_Drivebase.resetEncoders();
    }

}
