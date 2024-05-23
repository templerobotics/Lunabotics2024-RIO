package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivebase;

public class AutoDriveTest extends ParallelCommandGroup  {
    public AutoDriveTest(Drivebase drive) {
        System.out.println("AutoDriveTest");
        addCommands(
            new AutoDriveMove(drive)
        );
    }
}
