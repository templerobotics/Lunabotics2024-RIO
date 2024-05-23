package frc.robot.commands.jaredauto;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.Dumping.LinearActuatorState;
import frc.robot.subsystems.Digging.DiggingLinearActuator;
import frc.robot.subsystems.Digging.DiggingUppies;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DumpServo;
import frc.robot.subsystems.Digging.DiggingLinearActuator.*;
import edu.wpi.first.wpilibj2.command.*;


public class AutoDumpClose extends CommandBase {
    private DiggingLinearActuator m_DiggingLinear;
    private Dumping m_Dumping;
    private DumpServo m_DumpServo;
    private boolean sequenceIsDone;
    private final Timer timer = new Timer();

    public AutoDumpClose(DiggingLinearActuator diggingLinearActuator, Dumping dumpingActuator, DumpServo dumpServo) {
        m_DiggingLinear = diggingLinearActuator;
        m_Dumping = dumpingActuator;
        m_DumpServo = dumpServo;
        sequenceIsDone = false;

        addRequirements(m_DiggingLinear, m_Dumping, m_DumpServo);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute() {
        // double currentTime = timer.get();
        LinearActuatorState currentDumpingState = m_Dumping.getLinearState();
        LinearActuatorStateRight currentDiggingStateRight = m_DiggingLinear.linearActuatorStateRight();
        LinearActuatorStateLeft currentDiggingStateLeft = m_DiggingLinear.linearActuatorStateLeft();


        while (currentDumpingState != LinearActuatorState.Lowered) {
            m_Dumping.linearActuator(LinearActuatorState.Lowered);
            currentDumpingState = m_Dumping.getLinearState();
        }
        // System.out.println("DumpingLinearAcutator State: "  + currentDumpingState);
        // System.out.println("DiggingLinearActuatorStateRight: " + currentDiggingStateRight);
        // System.out.println("DiggingLinearActuatorStateLeft: " + currentDiggingStateLeft);
        // System.out.println("Current Time: " + currentTime);
        while (currentDumpingState == LinearActuatorState.Lowered && currentDiggingStateRight == LinearActuatorStateRight.Raised && currentDiggingStateLeft == LinearActuatorStateLeft.Raised) {
            // System.out.println("Hello I am able to actuate down wahoo!");
            m_DiggingLinear.linearActuatorInitStart();
            currentDiggingStateRight = m_DiggingLinear.linearActuatorStateRight();
            currentDiggingStateLeft = m_DiggingLinear.linearActuatorStateLeft();

        }
        // new SequentialCommandGroup(
        //     // new InstantCommand(() -> m_DiggingLinear.linearActuatorInitStart(), m_DiggingLinear),
        //     new WaitCommand(5),  // Wait for 8 seconds
        //     new InstantCommand(() -> m_DumpServo.servoClockwise(), m_DumpServo)).schedule();
        sequenceIsDone = true;
    }


    @Override
    public void end(boolean interrupted) {
            m_DumpServo.servoClockwise();
            // System.out.println("hellooooo");
    }

    @Override
    public boolean isFinished() {
        return sequenceIsDone;
    }
}