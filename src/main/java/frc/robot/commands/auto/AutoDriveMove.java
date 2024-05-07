package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class AutoDriveMove extends CommandBase {
    Drivebase m_Drivebase;
    private int m_timer;
    

    public AutoDriveMove(Drivebase driver) {
        m_Drivebase = driver;
        addRequirements(driver);
    }

    @Override
    public void initialize() {
        m_timer = 0;
        System.out.println("hello world");
        m_Drivebase.drive(1, 1);
    }

    @Override
    public void execute() {
        if (m_timer > 0) m_timer++;
    }

    @Override
    public void end(boolean interrupted) {
        m_Drivebase.drive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return (m_timer > 20);
    }

}
