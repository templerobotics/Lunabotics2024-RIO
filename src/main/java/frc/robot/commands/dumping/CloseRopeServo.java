package frc.robot.commands.dumping;

import frc.robot.subsystems.DumpServo;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseRopeServo extends CommandBase
{

    private final DumpServo m_subsystem;
    private int m_timer = 0;


    /**
     * Creates a new OpenRopeServo command.
     *
     * @param subsystem The subsystem used by this command.
     */

    public CloseRopeServo(DumpServo subsystem)
    {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() 
    {
        m_timer = 0;
    }

    @Override
    public void execute()
    {
        if(m_timer < 250)
        {
            m_subsystem.servoCClockwise();
        }
        m_timer++;
    }

    public void end(boolean interrupted)
    {
        m_subsystem.stopRope();
    }


    @Override
    public boolean isFinished()
    {
        return m_timer >= 250;
    }

}
