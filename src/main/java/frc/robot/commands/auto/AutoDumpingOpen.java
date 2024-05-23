// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Dumping;
// import frc.robot.subsystems.GearServo;
// import frc.robot.subsystems.Digging.DiggingLinearActuator;
// import frc.robot.subsystems.DumpServo;
// import frc.robot.commands.dumping.OpenRopeServo;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import java.util.concurrent.atomic.*;

// public class AutoDumpingOpen extends CommandBase {
//     Dumping m_Dumping;
//     DumpServo m_DumpServo;
//     GearServo m_GearServo;
//     DiggingLinearActuator m_LinearActuator;
//     private long startTime;
//     private AtomicBoolean active;

//     public AutoDumpingOpen(Dumping dumping, DumpServo dumpServo, DiggingLinearActuator diggingLinearActuator) {
//         m_Dumping = dumping;
//         m_DumpServo = dumpServo;
//         m_LinearActuator = diggingLinearActuator;
//         active = new AtomicBoolean(true);

//         addRequirements(dumping);
//         addRequirements(dumpServo);
//         addRequirements(diggingLinearActuator);

        
//     }
    

//     @Override
//     public void initialize() {
//     }

//     public void stop() {
//         active.set(false);
//     }

//     @Override
//     public void execute() {
//         startTime = System.currentTimeMillis();
//         if (!isFinished()) {
//             m_LinearActuator.linearActuatorInitStartAuto();
//             try {
//                 Thread.sleep(4000); // Wait for linear actuator initialization
//             } catch (InterruptedException e) {
//                 Thread.currentThread().interrupt();
//             }
//             m_LinearActuator.linearActuatorInitEndAutoDigging();
//             m_DumpServo.servoCClockwise();
//             try {
//                 Thread.sleep(500); // Wait for linear actuator initialization
//             } catch (InterruptedException e) {
//                 Thread.currentThread().interrupt();
//             }
//             m_DumpServo.stop();
//             m_DumpServo.stopRope();

//             m_Dumping.initializeUp();

//         }
//     }

//     @Override
//     public void end(boolean interrupted) 
//     {
//         m_DumpServo.stopRope();
//     }

//     @Override
//     public boolean isFinished() {
//         return System.currentTimeMillis() - startTime >= 25000; // Finish after 25 seconds
//     }
// }


package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.DumpServo;
import frc.robot.subsystems.Digging.DiggingLinearActuator;

public class AutoDumpingOpen extends CommandBase {
    private final Dumping m_Dumping;
    private final DumpServo m_DumpServo;
    private final DiggingLinearActuator m_LinearActuator;
    private final Timer timer = new Timer();
    private boolean servoActivated;

    public AutoDumpingOpen(Dumping dumping, DumpServo dumpServo, DiggingLinearActuator diggingLinearActuator) {
        m_Dumping = dumping;
        m_DumpServo = dumpServo;
        m_LinearActuator = diggingLinearActuator;

        addRequirements(dumping, dumpServo, diggingLinearActuator);
    }

    @Override
    public void initialize() {
        m_LinearActuator.linearActuatorInitStartAuto();
        timer.reset();
        timer.start();
        servoActivated = false;
    }

    @Override
    public void execute() {
        if (timer.get() < 2.0) {
            // Wait for 2 seconds before starting servo action
            return;
        }

        if (!servoActivated) {
            m_DumpServo.servoCClockwise();
            servoActivated = true;
            timer.reset();  // Reset the timer to time the servo action
        }


        if (timer.get() >= 0.1 && servoActivated) {
            m_DumpServo.stopRope();
            m_LinearActuator.linearActuatorInitEndAutoDigging();
            m_Dumping.commandUp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_DumpServo.stopRope();
        if (interrupted) {
            m_LinearActuator.linearStopMoving();
            m_Dumping.commandStop();

        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 15.0; // Finish after 15 seconds
    }
}
