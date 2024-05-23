package frc.robot.commands.jaredauto;

import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveToBurm extends CommandBase {
    private Drivebase m_driveBase;
    private final Timer timer = new Timer();
    private static final double DRIVE_SPEED = -0.5; // Speed at which the robot should drive backwards

    public AutoDriveToBurm(Drivebase drivebase) {
        m_driveBase = drivebase;
        addRequirements(m_driveBase);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_driveBase.drive(DRIVE_SPEED, DRIVE_SPEED); // Drive forward at a consistent speed
    }

    @Override
    public void end(boolean interrupted) {
        m_driveBase.drive(0, 0); // Stop the robot
        timer.stop();
    }

    @Override 
    public boolean isFinished() {
        return timer.hasElapsed(8); // Finish after 8 seconds
    }
}
