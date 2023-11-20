/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.custom.LunaMathUtils;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Drivebase.ScalarFunction;
// import frc.robot.subsystems.digging.DiggingLeadscrew.LeadscrewStatus;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.ControllerConstants.*;

/**
 * An example command that uses an example subsystem.
 */
public class XboxDrive extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Drivebase m_drivebase;
	private XboxController m_xbox;
	// private LeadscrewStatus m_leadscrew;

	double leftSpeed;
	double rightSpeed;
	boolean pirouetting;
	double turn;
	double throttle;

	/**
	 * Creates a new XboxDrive.
	 *
	 * @param subsystem The subsystem used by this command.
	 * @param xController The Xbox Controller used to control the drivebase
	 */
	public XboxDrive(Drivebase subsystem, XboxController xController/*, LeadscrewStatus leadscrewStatus*/) {
		m_drivebase = subsystem;
		m_xbox = xController;
		// m_leadscrew = leadscrewStatus;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		leftSpeed = 0;
		rightSpeed = 0;

		//pirouetting = m_xbox.getStickButton(m_xbox.kLeftStick);
        pirouetting = m_xbox.getLeftStickButton();
		//turn = m_xbox.getX(Hand.kRight);
		ScalarFunction scalarFunction = m_drivebase.getInputScaling();
		turn = pirouetting ? scalarFunction.calculate(m_xbox.getLeftX()) : m_xbox.getRightX()*0.5;
		throttle = scalarFunction.calculate(-1 * m_xbox.getLeftY()); // Invert throttle (up = forward)

		if (pirouetting && Math.abs(turn) > AXIS_DEADZONE) { // If turning in place
			leftSpeed = turn;
			rightSpeed = -1 * turn;
		} 
		else if (turn > AXIS_DEADZONE && Math.abs(throttle) > AXIS_DEADZONE) { // if we're heading right
			leftSpeed = throttle;
			rightSpeed = throttle * (1 - turn); // slow down the right by a proportional amount
		} 
		else if (turn < -AXIS_DEADZONE && Math.abs(throttle) > AXIS_DEADZONE) { // if we're heading left
			leftSpeed = throttle * (1 + turn); // slow down the left by a proportional amount
			rightSpeed = throttle;
		} 
		else if (Math.abs(throttle) > AXIS_DEADZONE) {
			leftSpeed = throttle;
			rightSpeed = throttle;
		}
		m_drivebase.drive(leftSpeed, rightSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_drivebase.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}