package frc.robot.commands.auto;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.Digging.*;
import frc.robot.Constants.GlobalConstants.RobotSide;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;


public class AutoDigging extends CommandBase
{
    DiggingBelt m_DiggingBelt;
    DiggingLeadscrew m_Leadscrew;
    DiggingLinearActuator m_LinearActuator;
    Drivebase m_Drivebase;
    Dumping m_Dumping;
    private final Timer timer = new Timer();



    public AutoDigging(DiggingBelt diggingBelt, DiggingLeadscrew diggingLeadscrew, DiggingLinearActuator diggingLinearActuator, Drivebase drivebase, Dumping dumping)
    {
        m_DiggingBelt = diggingBelt;
        m_Leadscrew = diggingLeadscrew;
        m_LinearActuator = diggingLinearActuator;
        m_Drivebase = drivebase;
        m_Dumping = dumping;

        addRequirements(diggingBelt);
        addRequirements(diggingLeadscrew);
        addRequirements(diggingLinearActuator);
        addRequirements(drivebase);
        addRequirements(dumping);
    }
    @Override
    public void initialize() 
    {
        m_LinearActuator.linearActuatorInitStartAuto();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() 
    {
        double currentTime = timer.get();

        // Start linear actuator initialization if it hasn't been initialized yet
        if (!m_LinearActuator.isLinearActuatorInitializedAutoDigging() && currentTime < 4.0) {
            m_LinearActuator.linearActuatorInitEndAutoDigging();
        }
        
        // Begin periodic operation after the initial delay of 4 seconds
        if (currentTime >= 4.0)
        {
            // Calculate the time since the periodic phase started
            double periodicTime = currentTime - 4.0;
            // Determine the current phase within each 4 second cycle
            double phase = periodicTime % 4.0;
    
            // Run belts if linear actuator is initialized
            if (m_LinearActuator.isLinearActuatorInitializedAutoDigging()) {
                m_DiggingBelt.runBelt(true);
            }
    
            // Control leadscrew based on the phase
            if (phase < 2.0) {
                m_Drivebase.drive(0.1, 0.1);
                // Check if the bottom limit switches are not pressed before moving the leadscrews down
                if (!m_Leadscrew.leadscrewStatus.bottomLimit()) {
                    m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Left, 0.25);
                    m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Right, 0.25);
                } else {
                    // If the bottom limit is reached, stop the leadscrews
                    m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Left, 0);
                    m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Right, 0);
                }
            } else {
                // Always stop the leadscrews in the off-phase of the cycle
                m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Left, 0);
                m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Right, 0);
                m_Drivebase.drive(0, 0);
            }
        }
    }


    @Override
    public void end(boolean interrupted) 
    {
        m_DiggingBelt.stopBelt();
        m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Left, -0.25);
        m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Right, -0.25);
        m_Drivebase.stop();
        new SequentialCommandGroup(
        new WaitCommand(8),  // Wait for 8 seconds
        new InstantCommand(() -> m_LinearActuator.linearActuatorInitStart(), m_LinearActuator)).schedule();
    }

    @Override
    public boolean isFinished() {
        return m_Dumping.getCurrentDraw() >= 3; // at about 102 lbs
    }
}