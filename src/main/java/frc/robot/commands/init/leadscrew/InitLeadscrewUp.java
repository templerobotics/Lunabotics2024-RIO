package frc.robot.commands.init.leadscrew;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GlobalConstants.RobotSide;
import frc.robot.subsystems.Digging.DiggingLeadscrew;
import static frc.robot.Constants.GlobalConstants.*;

public class InitLeadscrewUp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final DiggingLeadscrew m_subsystem;

    private boolean lInit;
    private boolean rInit;

    public InitLeadscrewUp(DiggingLeadscrew subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        lInit = false;
        rInit = false;
        m_subsystem.leadscrewSetRawSpeed(RobotSide.Left, -0.25);
        m_subsystem.leadscrewSetRawSpeed(RobotSide.Right, -0.25);
    }

    @Override
    public void execute() {
        if (!lInit && m_subsystem.leadscrewGetRawLimitSwitch(RobotSide.Left, MechanismPosition.Top)){
            m_subsystem.leadscrewSetRawSpeed(RobotSide.Left, 0);
            lInit = true;
        }
        if(!rInit && m_subsystem.leadscrewGetRawLimitSwitch(RobotSide.Right, MechanismPosition.Top)){
            m_subsystem.leadscrewSetRawSpeed(RobotSide.Right, 0);
            rInit = true;
        }
        if(lInit && rInit) m_subsystem.leadscrewFinalizeInit();
    }

    // Called once the command ends or in interrupted.
    @Override
    public void end(boolean interrupted){
        m_subsystem.leadscrewSetRawSpeed(RobotSide.Left, 0);
        m_subsystem.leadscrewSetRawSpeed(RobotSide.Right, 0);
    }

    //Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return lInit && rInit;
    }

}