package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Digging.*;
import frc.robot.subsystems.Dumping;


public class AutoDiggingTest extends ParallelCommandGroup
{
    public AutoDiggingTest(DiggingBelt diggingBelt, DiggingLeadscrew diggingLeadscrew, DiggingLinearActuator diggingLinearActuator, Drivebase drivebase, Dumping dumping)
    {
        System.out.println("AutoDiggingTest");
        addCommands(
            new AutoDigging(diggingBelt, diggingLeadscrew, diggingLinearActuator, drivebase, dumping)
        );
    }
}
