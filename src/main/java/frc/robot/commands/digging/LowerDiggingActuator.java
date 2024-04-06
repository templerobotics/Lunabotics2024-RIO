package frc.robot.commands.digging;

import frc.robot.subsystems.Digging.DiggingLinearActuator;
import frc.robot.subsystems.Digging.DiggingLinearActuator.LinearActuatorState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerDiggingActuator extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final DiggingLinearActuator m_subsystem;

    /**
	 * Creates a new LowerLinearActuator.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public LowerDiggingActuator(DiggingLinearActuator subsystem) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

    // Called when the command is initially scheduled.
	@Override
	public void initialize() {
		//m_subsystem.linearActuator(LinearActuatorState.Lowered);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_subsystem.linearActuator(LinearActuatorState.Lowered);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_subsystem.linearActuatorState() == LinearActuatorState.Lowered;
	}
}
