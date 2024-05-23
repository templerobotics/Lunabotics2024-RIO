package frc.robot.commands.dumping;

import frc.robot.subsystems.DumpServo;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseRopeServo extends CommandBase {
    private final DumpServo m_subsystem;
    private int m_timer = 0;  // Timer to keep track of the elapsed time in ticks

    /**
     * Creates a new OpenRopeServo command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public CloseRopeServo(DumpServo subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
            m_subsystem.servoClockwise();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopRope();  // Stop the servo when the command ends or is interrupted
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}