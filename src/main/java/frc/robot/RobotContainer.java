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
import frc.robot.commands.init.InitLinearActuator;
import frc.robot.subsystems.Digging.DiggingBelt;
import frc.robot.commands.OperatorDrive;
import frc.robot.commands.digging.ExtendLeadscrew;
import frc.robot.commands.digging.LeadscrewSetpoint;

import frc.robot.commands.digging.OperatorDigging;
import frc.robot.commands.digging.RetractLeadscrew;
import frc.robot.commands.digging.DigForward;
import frc.robot.commands.digging.DigReverse;
import frc.robot.subsystems.Digging.DiggingLeadscrew;
import static frc.robot.ButtonMapping.*;
import frc.robot.commands.init.leadscrew.InitLeadscrewDown;
import frc.robot.commands.init.leadscrew.InitLeadscrewUp;
import frc.robot.commands.dumping.LowerDumpingActuator;
// Dumping
import frc.robot.commands.dumping.RaiseDumpingActuator;
import frc.robot.subsystems.Dumping;

public class RobotContainer {
    private static final int OperatorDigging = 0;
    // Subsystems
    private final Drivebase s_Drivebase = new Drivebase();
    // private final Dumping s_Dumping = new Dumping();
    private final DiggingLeadscrew s_DiggingLeadscrew = new DiggingLeadscrew();
    private final DiggingBelt s_DiggingBelt = new DiggingBelt();
	private final Dumping s_DumpingLinearActuator = new Dumping();


    // Input Devices
    private final XboxController i_driverXbox = new XboxController(XBOX_CONTROLLER_DRIVER);
    private final XboxController i_operatorXbox = new XboxController(XBOX_CONTROLLER_OPERATOR);

    // Commands
    private final XboxDrive c_XboxDrive = new XboxDrive(s_Drivebase, i_driverXbox);
    private final OperatorDrive c_OperatorDrive = new OperatorDrive(s_DiggingLeadscrew, i_operatorXbox);
    private final DigForward c_DigForward = new DigForward(s_DiggingBelt);
	private final DigReverse c_DigReverse = new DigReverse(s_DiggingBelt);
    private final ExtendLeadscrew c_ExtendLeadscrew = new ExtendLeadscrew(s_DiggingLeadscrew);
	private final RetractLeadscrew c_RetractLeadscrew = new RetractLeadscrew(s_DiggingLeadscrew);
	private final LeadscrewSetpoint c_LeadscrewSetpoint = new LeadscrewSetpoint(s_DiggingLeadscrew);
    private final OperatorDigging c_OperatorDigging = new OperatorDigging(s_DiggingBelt, i_driverXbox);

    private final InitLeadscrewDown c_InitLeadscrewDown = new InitLeadscrewDown(s_DiggingLeadscrew);
	private final InitLeadscrewUp c_InitLeadscrewUp = new InitLeadscrewUp(s_DiggingLeadscrew);

	// Dumping
	private final RaiseDumpingActuator c_RaiseDumpingActuator = new RaiseDumpingActuator(s_DumpingLinearActuator);
	private final LowerDumpingActuator c_LowerDumpingActuator = new LowerDumpingActuator(s_DumpingLinearActuator);
	private final InitLinearActuator c_InitLinearActuator = new InitLinearActuator(s_DumpingLinearActuator);
	

    public RobotContainer() {
        s_Drivebase.resetEncoders();
        configureButtonBindings();
		Shuffleboard.getTab("Competition").getLayout("LSCommand").add("Command Setpoint", c_LeadscrewSetpoint);
    }

    private void configureButtonBindings() {

		POVButton operatorDPadUpButton = new POVButton(i_driverXbox, DumpUp); // D-Pad Up
		operatorDPadUpButton.onTrue(c_RaiseDumpingActuator);

		POVButton operatorDPadDownButton = new POVButton(i_driverXbox, DumpDown); // D-Pad Up
		operatorDPadDownButton.onTrue(c_LowerDumpingActuator);

        /*
		JoystickButton operatorXButton = new JoystickButton(i_operatorXbox, RaiseLinearActuator);
		operatorXButton.whenActive(c_RaiseLinearActuator);

		JoystickButton operatorBButton = new JoystickButton(i_operatorXbox, LowerLinearActuator);
		operatorBButton.whenActive(c_LowerLinearActuator);*/

		// JoystickButton operatorRStickButton = new JoystickButton(i_operatorXbox, XboxController.Button.kStickRight.value);
		// operatorRStickButton.whileHeld(c_AnalogLeadscrew);

        /*
		POVButton operatorDPadUpButton = new POVButton(i_driverXbox, DumpForward); // D-Pad Up
		operatorDPadUpButton.whileHeld(c_DumpForward);

		POVButton operatorDPadDownButton = new POVButton(i_driverXbox, DumpBackward); // D-Pad Down
		operatorDPadDownButton.whileHeld(c_DumpBackward);
        */

		/*JoystickButton operatorXButton = new JoystickButton(i_operatorXbox, RaiseLinearActuator);
		operatorXButton.whileTrue(c_RaiseLinearActuator);

		JoystickButton operatorBButton = new JoystickButton(i_operatorXbox, LowerLinearActuator);
		operatorBButton.whileTrue(c_LowerLinearActuator);
        */
		POVButton operatorDPadRightButton = new POVButton(i_operatorXbox, OperatorDrive); // D-Pad Right
		operatorDPadRightButton.whileTrue(c_OperatorDrive);

		POVButton operatorDigging = new POVButton(i_operatorXbox, OperatorDigging);
		operatorDigging.whileTrue(c_OperatorDigging);

		JoystickButton operatorYButton = new JoystickButton(i_operatorXbox, DigForward);
		operatorYButton.toggleOnTrue(c_DigForward);

		JoystickButton operatorAButton = new JoystickButton(i_operatorXbox, DigReverse);
		operatorAButton.toggleOnTrue(c_DigReverse);

		JoystickButton operatorRBumper = new JoystickButton(i_operatorXbox, RetractLeadscrew);
		operatorRBumper.onTrue(c_RetractLeadscrew);

		JoystickButton operatorLBumper = new JoystickButton(i_operatorXbox, ExtendLeadscrew);
		operatorLBumper.onTrue(c_ExtendLeadscrew);
	}

    public Command getXboxDrive() {
		return c_XboxDrive;   
	}

    public Command getOperatorDrive(){
		return c_OperatorDrive;
	}

    public Command getInitializeLeadscrewCommand(){
		return new WaitCommand(0.2).andThen(c_InitLeadscrewDown).andThen(new WaitCommand(1)).andThen(c_InitLeadscrewUp);
	}

    
	public Command getInitializeLinearCommand(){
		return new WaitCommand(0.2).andThen(c_InitLinearActuator);
	}

    public boolean isLeadscrewInitialized(){
		return s_DiggingLeadscrew.isLeadscrewInitialized();
	}
    
	public boolean isLinearActuatorInitialized(){
		return s_DumpingLinearActuator.isLinearActuatorInitialized();
	}
    
}