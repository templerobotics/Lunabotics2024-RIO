package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.GearServo;
import frc.robot.subsystems.Digging.DiggingLinearActuator;
import frc.robot.subsystems.DumpServo;


public class AutoDumping extends CommandBase
{
    Dumping m_Dumping;
    DumpServo m_DumpServo;
    GearServo m_GearServo;
    DiggingLinearActuator m_LinearActuator;
    private long startTime;

    public AutoDumping(Dumping dumping, DumpServo dumpServo, GearServo gearServo, DiggingLinearActuator diggingLinearActuator)
    {
        m_Dumping = dumping;
        m_DumpServo = dumpServo;
        m_GearServo = gearServo;
        m_LinearActuator = diggingLinearActuator;

        addRequirements(dumping);
        addRequirements(dumpServo);
        addRequirements(gearServo);
        addRequirements(diggingLinearActuator);
    }

    @Override
    public void initialize() 
    {

    }

    @Override
    public void execute()
    {
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
        }
        while(!isFinished())
        {
            m_LinearActuator.linearActuatorInitStart();
            try {
                Thread.sleep(4000); // Wait for linear actuator initialization
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            m_DumpServo.servoClockwise();
            m_GearServo.servoClockwise();
            m_Dumping.commandUp();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        m_Dumping.linearActuatorInitStart();
        m_DumpServo.servoCClockwise();
        m_GearServo.servoCClockwise();
        try {
            Thread.sleep(8000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        m_LinearActuator.linearActuatorInitStart();

    }

    @Override
    public boolean isFinished() 
    {
        return System.currentTimeMillis() - startTime >= 25000; // Finish after 20 seconds
    }

}
