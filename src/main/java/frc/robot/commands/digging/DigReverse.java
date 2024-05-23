package frc.robot.commands.digging;

import frc.robot.subsystems.Digging.DiggingBelt;
import edu.wpi.first.wpilibj2.command.CommandBase;;

public class DigReverse extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final DiggingBelt m_subsystem;

    // Creates a new DigReverse Object
    // @param subsystem: THe subsystem used by this command
    public DigReverse(DiggingBelt subsystem){
        m_subsystem = subsystem;
        //Decalre subsystem dependencies
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while command is scheduled
    @Override
    public void execute(){
        m_subsystem.runBelt(true); //Run Belt in reverse
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