package frc.robot.commands.digging;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Digging.DiggingBelt;

public class OperatorDigging extends CommandBase {
    private final DiggingBelt m_digging;
    private XboxController m_xbox;

    public OperatorDigging(DiggingBelt subsystem, XboxController xController){
        m_digging = subsystem;
        m_xbox = xController;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double digSpeed = 0;
        //Read Inputs from controller
        boolean rightTrigger = m_xbox.getRightBumperPressed(); //This is a new command since Hand class was depricated
        boolean leftTrigger = m_xbox.getLeftBumperPressed(); ////This is a new command since Hand class was depricated

        if (rightTrigger){
            digSpeed = digSpeed + 100; //increase dig speed
        }else if(leftTrigger){
            digSpeed = digSpeed - 100; //decrease dig speed
        }
        m_digging.setBeltSpeed(digSpeed); // Sets the dig speed of the belt
    }

    // Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

}