// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.jaredauto.AutoDumpClose;
import frc.robot.commands.jaredauto.AutoDumpOpen;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.DumpServo;
import frc.robot.subsystems.Dumping;
import frc.robot.subsystems.Digging.DiggingLinearActuator;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoDumpingTest extends ParallelCommandGroup {
  /** Creates a new AutoDumpingTest. */
  public AutoDumpingTest(AutoDumpOpen autoDumpingOpen, AutoDumpClose autoDumpingClose, Drivebase drivebase, Dumping dumping, DumpServo dumpServo, DiggingLinearActuator diggingLinearActuator) 
  {
      addCommands(
        new AutoDumping(autoDumpingOpen, autoDumpingClose, drivebase, dumping, dumpServo, diggingLinearActuator)
      );
  }
}
