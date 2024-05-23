package frc.robot.commands.digging;

import frc.robot.subsystems.Digging.DiggingBelt;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DigForward extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final DiggingBelt m_subsystem;

    // Creates a new DigForward Object
    // @param subsystem: THe subsystem used by this command
    public DigForward(DiggingBelt subsystem){
        m_subsystem = subsystem;
        //Decalre subsystem dependencies
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while command is scheduled
    @Override
    public void execute(){
        m_subsystem.runBelt(false); //Run Belt
    }

    // Called when command ends or is interupted
    @Override
    public void end(boolean interupted){
        m_subsystem.stopBelt(); //Stops belt
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}