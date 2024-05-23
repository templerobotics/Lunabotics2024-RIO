package frc.robot.commands.digging;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Digging.DiggingLeadscrew;
import frc.robot.subsystems.Digging.DiggingLeadscrew.LeadscrewState;


public class RetractLeadscrew extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final DiggingLeadscrew m_subsystem;


    public RetractLeadscrew(DiggingLeadscrew subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

        m_subsystem.leadscrew(LeadscrewState.Retracted);

    }
    
    @Override
	public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_subsystem.leadscrewState() == LeadscrewState.Retracted;
    }
}