package frc.robot.commands.auto;

import frc.robot.subsystems.Digging.*;
import frc.robot.Constants.GlobalConstants.RobotSide;
import edu.wpi.first.wpilibj2.command.*;


public class AutoDigging extends CommandBase
{
    DiggingBelt m_DiggingBelt;
    DiggingLeadscrew m_Leadscrew;
    DiggingLinearActuator m_LinearActuator;


    public AutoDigging(DiggingBelt diggingBelt, DiggingLeadscrew diggingLeadscrew, DiggingLinearActuator diggingLinearActuator)
    {
        m_DiggingBelt = diggingBelt;
        m_Leadscrew = diggingLeadscrew;
        m_LinearActuator = diggingLinearActuator;

        addRequirements(diggingBelt);
        addRequirements(diggingLeadscrew);
        addRequirements(diggingLinearActuator);
    }
    @Override
    public void initialize() 
    {

    }

    private long startTime;


    @Override
public void execute() {

    if (startTime == 0) {
        startTime = System.currentTimeMillis();
    }
    while (!isFinished()) 
    { 
        m_LinearActuator.linearActuatorInitStartAuto();
    try {
        Thread.sleep(4000); // Wait for linear actuator initialization
    } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
    }
    m_LinearActuator.linearActuatorInitEndAutoDigging();
    if (m_LinearActuator.isLinearActuatorInitializedAutoDigging()) {
        m_DiggingBelt.runBelt(true);
    }
// Run for a total of 10 seconds
        long loopStartTime = System.currentTimeMillis();

        // Run the leadscrews for 4 seconds
        while (System.currentTimeMillis() - loopStartTime < 4000) {
            m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Left, 0.25);
            m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Right, 0.25);
        }

        // Stop the leadscrews
        m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Left, 0);
        m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Right, 0);

        // Sleep to maintain 4-second interval
        long elapsedTime = System.currentTimeMillis() - startTime;
        long sleepTime = Math.max(0, 4000 - elapsedTime % 4000);
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}


    @Override
    public void end(boolean interrupted) 
    {
        m_DiggingBelt.stopBelt();
        m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Left, -0.25);
        m_Leadscrew.leadscrewSetRawSpeed(RobotSide.Right, -0.25);
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        m_LinearActuator.linearActuatorInitStart();

    }

    @Override
public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= 20000; // Finish after 20 seconds
}
}