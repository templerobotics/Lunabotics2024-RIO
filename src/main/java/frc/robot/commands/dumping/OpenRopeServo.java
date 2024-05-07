package frc.robot.commands.dumping;

import frc.robot.subsystems.DumpServo;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenRopeServo extends CommandBase {
    private final DumpServo m_subsystem;
    private int m_timer = 0;  // Timer to keep track of the elapsed time in ticks

    /**
     * Creates a new OpenRopeServo command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public OpenRopeServo(DumpServo subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_timer < 250) {  // 250 ticks at 20 ms per tick is 5 seconds
            m_subsystem.servoClockwise();
        }
        m_timer++;  // Increment the timer on each execution
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopRope();  // Stop the servo when the command ends or is interrupted
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer >= 250;  // End the command after 5 seconds
    }
}
