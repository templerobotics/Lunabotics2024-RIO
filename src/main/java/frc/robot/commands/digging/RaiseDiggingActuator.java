/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands.digging;


import frc.robot.subsystems.Digging.DiggingLinearActuator;
import frc.robot.subsystems.Digging.DiggingLinearActuator.LinearActuatorStateRight;
import frc.robot.subsystems.Digging.DiggingLinearActuator.LinearActuatorStateLeft;


import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * An example command that uses an example subsystem.
 */
public class RaiseDiggingActuator extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DiggingLinearActuator m_subsystem;
    private final DiggingLinearActuator m_subsystem2;



    /**
     * Creates a new RaiseLinearActuator.
     *
     * @param subsystem The subsystem used by this command.
     */
    public RaiseDiggingActuator(DiggingLinearActuator subsystem) {
        m_subsystem = subsystem;
        m_subsystem2 = subsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.linearActuatorRight(LinearActuatorStateRight.Raised);
        m_subsystem2.linearActuatorLeft(LinearActuatorStateLeft.Raised);


    }
    


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_subsystem.linearActuatorStateRight() == LinearActuatorStateRight.Raised && m_subsystem2.linearActuatorStateLeft() == LinearActuatorStateLeft.Raised;
    }
}
