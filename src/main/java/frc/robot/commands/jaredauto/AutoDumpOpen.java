package frc.robot.commands.jaredauto;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.Digging.*;
import frc.robot.subsystems.Digging.DiggingLinearActuator.LinearActuatorLimits;
import frc.robot.Constants.GlobalConstants.RobotSide;
import frc.robot.commands.digging.SetDiggingActuatorMaxTravel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DumpServo;
import frc.robot.subsystems.Dumping.LinearActuatorState;

import edu.wpi.first.wpilibj.Timer;

public class AutoDumpOpen extends CommandBase {
    DiggingLinearActuator m_DiggingLinear;
    Dumping m_Dumping;
    DumpServo m_DumpServo;
    private final Timer timer = new Timer();
    private final Timer servoTimer = new Timer();  // Additional timer for servo operation
    private boolean sequenceIsDone;
    private boolean servoActive;  // Flag to check if servo operation is ongoing

    public AutoDumpOpen(DiggingLinearActuator diggingLinearActuator, Dumping dumpingActuator, DumpServo dumpServo) {
        m_DiggingLinear = diggingLinearActuator;
        m_Dumping = dumpingActuator;
        m_DumpServo = dumpServo;
        sequenceIsDone = false;
        servoActive = false;  // Initialize the servo operation flag

        addRequirements(m_DiggingLinear, m_Dumping, m_DumpServo);
    }

    @Override
    public void initialize() {
        m_DiggingLinear.setMaxTravel(0.7);
        m_DiggingLinear.linearActuatorInitStartAuto();
        timer.reset();
        timer.start();
        servoTimer.stop();  // Ensure the servo timer is stopped at initialization
        servoTimer.reset();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();

        if (!servoActive) {
            servoTimer.start();
            while(timer.get() < 0.7
            
            )
            {
                m_DumpServo.servoCClockwise();
            }
            servoActive = true;
        }
    
        m_DumpServo.stopServo();
        if (servoActive) {
            servoTimer.stop();
            servoTimer.reset();
            servoActive = false;
        }
        if (m_DiggingLinear.isLinearActuatorInitializedAutoDigging() && currentTime < 4.0) {
            m_DiggingLinear.linearActuatorInitEndAutoDigging();
        }
        if (m_Dumping.isLinearActuatorInitialized() && currentTime >= 4) {
            m_Dumping.linearActuator(LinearActuatorState.Raised);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && servoActive) {
            m_DumpServo.stopServo();
            servoTimer.stop();
            servoTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return sequenceIsDone;
    }
}
