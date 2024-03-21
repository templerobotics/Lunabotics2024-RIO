package frc.robot.commands.init.leadscrew;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GlobalConstants.RobotSide;
import frc.robot.subsystems.digging.DiggingLeadscrew;

public class InitLeadscrewDown extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DiggingLeadscrew m_subsystem;

    private double time;

    public InitLeadscrewDown(DiggingLeadscrew subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.leadscrewClearFollower();
        m_subsystem.leadscrewSetRawSpeed(RobotSide.Left, 0.25);
        m_subsystem.leadscrewSetRawSpeed(RobotSide.Right, 0.25);
        time = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.leadscrewSetRawSpeed(RobotSide.Left, 0);
        m_subsystem.leadscrewSetRawSpeed(RobotSide.Right, 0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - time > 5;
    }



}