package frc.robot.commands;

import frc.robot.subsystems.Digging.DiggingLeadscrew;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.ControllerConstants.*;

public class OperatorDrive extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DiggingLeadscrew m_leadscrew;
	private XboxController m_xbox;

    public OperatorDrive(DiggingLeadscrew subsystem, XboxController xController) {
        m_leadscrew = subsystem;
        m_xbox = xController;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double leadscrewSpeed = 0;
        double leftTrigger = m_xbox.getLeftTriggerAxis();
        double rightTrigger = m_xbox.getRightTriggerAxis();

        if (leftTrigger > AXIS_DEADZONE && rightTrigger > AXIS_DEADZONE) {
            leadscrewSpeed = 0;
        }
        else if (leftTrigger > AXIS_DEADZONE){
			leadscrewSpeed = -1*leftTrigger;
		}
		else if (rightTrigger > AXIS_DEADZONE){
			leadscrewSpeed = rightTrigger;
		}
		m_leadscrew.leadscrewSpeed(MathUtil.clamp(leadscrewSpeed, -1, 0.5));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}