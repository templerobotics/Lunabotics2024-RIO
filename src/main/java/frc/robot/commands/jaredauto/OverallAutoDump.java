package frc.robot.commands.jaredauto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.DumpServo;
import frc.robot.subsystems.Digging.DiggingLinearActuator;

public class OverallAutoDump extends SequentialCommandGroup {
    
    public OverallAutoDump(DiggingLinearActuator diggingActuator, Dumping dumpingSystem, DumpServo dumpServo, Drivebase drivebase) {
        addCommands(
            new AutoDriveToBurm(drivebase),
            new AutoDumpOpen(diggingActuator, dumpingSystem, dumpServo)
        );
    }
}
