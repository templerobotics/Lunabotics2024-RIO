// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Dumping;
// import frc.robot.subsystems.GearServo;
// import frc.robot.subsystems.Digging.DiggingLinearActuator;
// import frc.robot.subsystems.DumpServo;
// import frc.robot.commands.dumping.CloseRopeServo;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;

// public class AutoDumpingClose extends CommandBase {
//     Dumping m_Dumping;
//     DumpServo m_DumpServo;
//     DiggingLinearActuator m_LinearActuator;
//     private long startTime;

//     public AutoDumpingClose(Dumping dumping, DumpServo dumpServo, DiggingLinearActuator diggingLinearActuator) {
//         m_Dumping = dumping;
//         m_DumpServo = dumpServo;
//         m_LinearActuator = diggingLinearActuator;

//         addRequirements(dumping);
//         addRequirements(dumpServo);
//         addRequirements(diggingLinearActuator);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         startTime = System.currentTimeMillis();
//         if (!isFinished()) {
//             m_Dumping.initializeDown();
//             try {
//                 Thread.sleep(5000); // Wait for linear actuator initialization
//             } catch (InterruptedException e) {
//                 Thread.currentThread().interrupt();
//             }
//             m_DumpServo.servoClockwise();
//             try {
//                 Thread.sleep(1000); // Wait for linear actuator initialization
//             } catch (InterruptedException e) {
//                 Thread.currentThread().interrupt();
//             }
//             m_DumpServo.stopServo();
//             m_LinearActuator.linearActuatorInitStartAuto();
//             try {
//                 Thread.sleep(4000); // Wait for linear actuator initialization
//             } catch (InterruptedException e) {
//                 Thread.currentThread().interrupt();
//             }
//             m_LinearActuator.linearActuatorInitEndAutoDigging();
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

public class AutoDumpingClose extends CommandBase {
    private final Dumping m_Dumping;
    private final DumpServo m_DumpServo;
    private final DiggingLinearActuator m_LinearActuator;
    private final Timer timer = new Timer();
    private boolean servoActivated;

    public AutoDumpingClose(Dumping dumping, DumpServo dumpServo, DiggingLinearActuator diggingLinearActuator) {
        m_Dumping = dumping;
        m_DumpServo = dumpServo;
        m_LinearActuator = diggingLinearActuator;

        addRequirements(dumping, dumpServo, diggingLinearActuator);
    }

    @Override
    public void initialize() {
        m_Dumping.commandDown();
        timer.reset();
        timer.start();
        servoActivated = false;
    }

    @Override
    public void execute() {
        if (timer.get() < 4.0) {
            // Wait for 4 seconds before starting servo action
            return;
        }

        if (!servoActivated) {
            m_DumpServo.servoClockwise();
            servoActivated = true;
            timer.reset();  // Reset the timer to time the servo action
            m_LinearActuator.commandDown();
        }

        if (timer.get() >= 0.5 && servoActivated) {
            m_DumpServo.stopRope();
            m_LinearActuator.linearStopMoving();
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
