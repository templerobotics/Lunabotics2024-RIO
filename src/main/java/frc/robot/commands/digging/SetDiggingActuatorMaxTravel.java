package frc.robot.commands.digging;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Digging.DiggingLinearActuator;
 
public class SetDiggingActuatorMaxTravel extends CommandBase {
    private final DiggingLinearActuator diggingActuator;
    private final double maxTravel;
 
    public SetDiggingActuatorMaxTravel(DiggingLinearActuator diggingActuator, double maxTravel) {
        this.diggingActuator = diggingActuator;
        this.maxTravel = maxTravel;
        addRequirements(diggingActuator);
    }
 
    @Override
    public void initialize() {
        // Set the maximum travel distance
        diggingActuator.setMaxTravel(maxTravel);
        diggingActuator.commandUp();
    }
 
    @Override
    public boolean isFinished() {
        return true; // Finish immediately after setting the maximum travel distance
    }
}