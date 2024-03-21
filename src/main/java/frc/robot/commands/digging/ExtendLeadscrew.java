package frc.robot.commands.digging;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.digging.DiggingLeadscrew;
import frc.robot.subsystems.digging.DiggingLeadscrew.LeadscrewState;

public class ExtendLeadscrew extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final DiggingLeadscrew m_subsystem;

    public ExtendLeadscrew(DiggingLeadscrew subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.leadscrew(LeadscrewState.Extended);
    }

    @Override
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_subsystem.leadscrewState() == LeadscrewState.Extended;
    }

}