package frc.robot.commands.digging;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.digging.DiggingLeadscrew;

public class LeadscrewSetpoint extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DiggingLeadscrew m_subsystem; 

    public LeadscrewSetpoint(DiggingLeadscrew subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.commandLeadscrew(m_subsystem.getDashboardCommandedPosition());
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return true;}
}