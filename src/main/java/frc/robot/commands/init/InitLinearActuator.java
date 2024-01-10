/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.init;

import frc.robot.subsystems.Digging.DiggingLinearActuator;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.GlobalConstants.*;
import static frc.robot.Constants.DiggingConstants.LINEAR_MIN_TRAVEL;

/**
 * An example command that uses an example subsystem.
 */
public class InitLinearActuator extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final DiggingLinearActuator m_subsystem;

	private boolean flag = false;

	/**
	 * Creates a new InitLinearActuator.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public InitLinearActuator(DiggingLinearActuator subsystem) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_subsystem.linearActuatorInitStart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_subsystem.linearActuatorGetRawPotentiometer(RobotSide.Left) <= LINEAR_MIN_TRAVEL && 
				m_subsystem.linearActuatorGetRawPotentiometer(RobotSide.Right) <= LINEAR_MIN_TRAVEL){
					m_subsystem.linearActuatorInitEnd();
					flag = true;
		}
	} 

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return flag;
	}
}
