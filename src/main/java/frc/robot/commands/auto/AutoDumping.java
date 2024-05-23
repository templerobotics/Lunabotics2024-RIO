// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.jaredauto.AutoDumpClose;
import frc.robot.commands.jaredauto.AutoDumpOpen;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.DumpServo;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.Digging.DiggingLinearActuator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Dumping.LinearActuatorState;




public class AutoDumping extends CommandBase 
{
  AutoDumpOpen m_AutoDumpingOpen;
  AutoDumpClose m_AutoDumpingClose;
  Drivebase m_Drivebase;
  DiggingLinearActuator m_DiggingLinear;
  Dumping m_Dumping;
  DumpServo m_DumpServo;
  

  private final Timer timer = new Timer();
  private final Timer servoTimer = new Timer();  // Additional timer for servo operation
  private boolean servoActive;  // Flag to check if servo operation is ongoing

  
  public AutoDumping(AutoDumpOpen autoDumpingOpen, AutoDumpClose autoDumpingClose, Drivebase drivebase, Dumping dumping, DumpServo dumpServo, DiggingLinearActuator diggingLinearActuator) 
  {
    m_AutoDumpingOpen = autoDumpingOpen;
    m_AutoDumpingClose = autoDumpingClose;
    m_Drivebase = drivebase;
    m_DumpServo = dumpServo;
    m_DiggingLinear = diggingLinearActuator;
    addRequirements(m_Drivebase, m_DumpServo, m_DiggingLinear);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.stop();
    timer.reset();
    servoTimer.stop();
    servoTimer.reset();
    servoActive = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    timer.start();
    while(timer.get() >= 0 && timer.get() <= 2)
    {
      m_Drivebase.drive(-1, -1);
    }
    m_Drivebase.drive(0, 0);
    // double currentTime = timer.get();

    if (!servoActive) {
        servoTimer.start();
        while(timer.get() < 0.7)
        {
            m_DumpServo.servoCClockwise();
        }
        servoActive = true;
    }

    m_DumpServo.stopServo();
    if (servoActive) {
        servoTimer.stop();
        servoTimer.reset();
        servoActive = false;
    }
    while(!m_DiggingLinear.isLinearActuatorInitialized())
    {
      if (m_DiggingLinear.isLinearActuatorInitializedAutoDigging()) 
      {
          m_DiggingLinear.linearActuatorInitEndAutoDigging();
      }
      else
      {
        m_DiggingLinear.commandUp();
      }
      if (m_Dumping.isLinearActuatorInitialized() && servoTimer >= 4) 
      {
          m_Dumping.linearActuator(LinearActuatorState.Raised);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}