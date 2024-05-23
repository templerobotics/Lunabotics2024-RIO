package frc.robot.commands.dumping;

import frc.robot.subsystems.DumpServo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;



public class OpenRopeServo extends CommandBase
{

    private final DumpServo m_subsystem;
    private final Timer timer = new Timer();
    private boolean sequenceIsDone;


    /**
     * Creates a new OpenRopeServo command.
     *
     * @param subsystem The subsystem used by this command.
     */

    public OpenRopeServo(DumpServo subsystem)
    {
        m_subsystem = subsystem;
        addRequirements(subsystem);
        sequenceIsDone = false;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute()
    {
        double currentTime = timer.get();
        // if(currentTime < 1)
        // {
        //     new SequentialCommandGroup(
        //         new WaitCommand(1),  // Wait for 1 seconds
        //         new InstantCommand(() ->m_subsystem.servoCClockwise(), m_subsystem)).schedule();
        //     sequenceIsDone = true;
        // } else {
        //     sequenceIsDone = true;
        //     m_subsystem.stopRope();
        // }
        while(timer.get() < 0.5)
        {
            m_subsystem.servoCClockwise();
        }
        m_subsystem.stopServo();
        sequenceIsDone = true;
    }

    public void end(boolean interrupted)
    {
        if (interrupted) {
            m_subsystem.stopRope();
        }
    }


    @Override
    public boolean isFinished()
    {
        return sequenceIsDone;
    }

}

// package frc.robot.commands.dumping;

// import frc.robot.subsystems.DumpServo;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj.Timer;

// public class OpenRopeServo extends CommandBase
// {

//     private final DumpServo m_subsystem;


//     /**
//      * Creates a new OpenRopeServo command.
//      *
//      * @param subsystem The subsystem used by this command.
//      */

//     public OpenRopeServo(DumpServo subsystem)
//     {
//         m_subsystem = subsystem;
//         addRequirements(subsystem);
//     }

//     @Override
//     public void initialize() {}

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//             m_subsystem.servoCClockwise();

//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         m_subsystem.stopRope();  // Stop the servo when the command ends or is interrupted
//     }

//     @Override
//     public boolean isFinished() {
//         return false; 
//     }
// }
