package frc.robot.commands.digging;

import frc.robot.Constants.GlobalConstants.RobotSide;
import frc.robot.subsystems.Digging.DiggingLinearActuator;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DiggingConstants.LINEAR_DEADBAND;

/**
 * An example command that uses an example subsystem.
 */
public class LinearActuatorSetpoint extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DiggingLinearActuator m_subsystem;
	private double commandedSetpoint;

	/**
	 * Creates a new RaiseLinearActuator.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public LinearActuatorSetpoint(DiggingLinearActuator subsystem) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		commandedSetpoint = m_subsystem.getDashboardCommandedPosition();
		System.out.println("LASetpoint Init. Setpoint: " + m_subsystem.getDashboardCommandedPosition());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_subsystem.getLinearPosition(RobotSide.Left) - LINEAR_DEADBAND > commandedSetpoint
				|| m_subsystem.getLinearPosition(RobotSide.Right) - LINEAR_DEADBAND > commandedSetpoint) {
			//System.out.println("LASetpoint CommandDown");
			m_subsystem.commandDown();
		}
		if (m_subsystem.getLinearPosition(RobotSide.Left) - LINEAR_DEADBAND < commandedSetpoint
				|| m_subsystem.getLinearPosition(RobotSide.Right) - LINEAR_DEADBAND < commandedSetpoint) {
			//System.out.println("LASetpoint CommandUp");
			m_subsystem.commandUp();
		}
		if (Math.abs(m_subsystem.getLinearPosition(RobotSide.Left) - commandedSetpoint) <= LINEAR_DEADBAND
				|| Math.abs(m_subsystem.getLinearPosition(RobotSide.Right) - commandedSetpoint) <= LINEAR_DEADBAND) {
			//System.out.println("LASetpoint CommandStop");
			m_subsystem.commandStop();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_subsystem.commandStop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.abs(m_subsystem.getLinearPosition(RobotSide.Left) - commandedSetpoint) <= LINEAR_DEADBAND
				|| Math.abs(m_subsystem.getLinearPosition(RobotSide.Right) - commandedSetpoint) <= LINEAR_DEADBAND;
	}
}
