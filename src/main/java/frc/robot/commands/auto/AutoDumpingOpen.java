package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.GearServo;
import frc.robot.subsystems.Digging.DiggingLinearActuator;
import frc.robot.subsystems.DumpServo;
import frc.robot.commands.dumping.OpenRopeServo;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoDumpingOpen extends CommandBase {
    Dumping m_Dumping;
    DumpServo m_DumpServo;
    GearServo m_GearServo;
    DiggingLinearActuator m_LinearActuator;
    private long startTime;

    public AutoDumpingOpen(Dumping dumping, DumpServo dumpServo, DiggingLinearActuator diggingLinearActuator) {
        m_Dumping = dumping;
        m_DumpServo = dumpServo;
        m_LinearActuator = diggingLinearActuator;

        addRequirements(dumping);
        addRequirements(dumpServo);
        addRequirements(diggingLinearActuator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        startTime = System.currentTimeMillis();
        if (!isFinished()) {
            m_LinearActuator.linearActuatorInitStartAuto();
            try {
                Thread.sleep(4000); // Wait for linear actuator initialization
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            m_LinearActuator.linearActuatorInitEndAutoDigging();
            m_DumpServo.servoCClockwise();
            try {
                Thread.sleep(500); // Wait for linear actuator initialization
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            m_DumpServo.stopRope();

            m_Dumping.initializeUp();
            try {
                Thread.sleep(10000); // Wait for linear actuator initialization
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void end(boolean interrupted) 
    {
        m_DumpServo.stopRope();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= 12000; // Finish after 25 seconds
    }
}
