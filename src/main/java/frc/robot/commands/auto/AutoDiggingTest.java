package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Digging.*;


public class AutoDiggingTest extends ParallelCommandGroup
{
    public AutoDiggingTest(DiggingBelt diggingBelt, DiggingLeadscrew diggingLeadscrew, DiggingLinearActuator diggingLinearActuator)
    {
        System.out.println("AutoDiggingTest");
        addCommands(
            new AutoDigging(diggingBelt, diggingLeadscrew, diggingLinearActuator)
        );
    }
}
