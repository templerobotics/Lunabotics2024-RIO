package frc.robot.commands.dumping;

import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.Dumping.LinearActuatorState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseDumpingActuator extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Dumping m_subsystem;

    /**
	 * Creates a new RaiseLinearActuator.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public RaiseDumpingActuator(Dumping subsystem) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

    // Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_subsystem.linearActuator(LinearActuatorState.Raised);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_subsystem.linearActuatorState() == LinearActuatorState.Raised;
	}
}
