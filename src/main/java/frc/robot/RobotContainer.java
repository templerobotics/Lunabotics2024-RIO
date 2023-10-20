package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import static frc.robot.Constants.ControllerConstants.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.XboxDrive;
import frc.robot.subsystems.Drivebase;


public class RobotContainer {
    // Subsystems
    private final Drivebase s_Drivebase = new Drivebase();

    // Input Devices
    private final XboxController i_driverXbox = new XboxController(XBOX_CONTROLLER_DRIVER);
    private final XboxController i_operatorXbox = new XboxController(XBOX_CONTROLLER_OPERATOR);

    // Commands
    private final XboxDrive c_XboxDrive = new XboxDrive(s_Drivebase, i_driverXbox);

    public RobotContainer() {
        s_Drivebase.resetEncoders();
    }

    public Command getXboxDrive() {
		return c_XboxDrive;   
	}
}
