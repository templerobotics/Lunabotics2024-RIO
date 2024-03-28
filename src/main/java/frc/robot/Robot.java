// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.custom.LunaMathUtils;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private Command c_XboxDrive;
  private Command c_OperatorDrive;
	private Command c_InitializeLeadscrew;
  private Command c_InitializeLinearActuator;
  
  private RobotContainer m_robotContainer;

  private GenericEntry nt_FPGATimestamp;
  private Counter counter = new Counter(Counter.Mode.kTwoPulse);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_robotContainer = new RobotContainer();

    c_XboxDrive = m_robotContainer.getXboxDrive();
    c_OperatorDrive = m_robotContainer.getOperatorDrive();
    c_InitializeLeadscrew = m_robotContainer.getInitializeLeadscrewCommand();
    c_InitializeLinearActuator = m_robotContainer.getInitializeLinearCommand();
    //c_XboxDrive = m_robotContainer.getXboxDrive();
    //nt_FPGATimestamp = Shuffleboard.getTab("Competition").add("FPGA Time", LunaMathUtils.roundToPlace(Timer.getFPGATimestamp(), 2)).withSize(1, 1).withPosition(0, 0).getEntry();
  
    nt_FPGATimestamp = Shuffleboard.getTab("Competition").add("FPGA Time", 0).withSize(1, 1).withPosition(0, 0).getEntry();

   
    counter.setUpSource(0);
    counter.setDownSource(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    nt_FPGATimestamp.setDouble(LunaMathUtils.roundToPlace(Timer.getFPGATimestamp(), 2));
    SmartDashboard.putNumber("Hall Effect Sensor", counter.getRate());
  }

  @Override
  public void disabledInit() {}

  @Override 
  public void disabledPeriodic(){}

  /** This function is run once each time the robot enters autonomous mode. */
  /*
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }
  */

  /** This function is called periodically during autonomous. */
  /*
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }
  */

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    c_XboxDrive.schedule();
    c_OperatorDrive.schedule();
		if (!m_robotContainer.isLeadscrewInitialized()) c_InitializeLeadscrew.schedule();
    if (!m_robotContainer.isLinearActuatorInitialized()) c_InitializeLinearActuator.schedule();
  }

  /** This function is called periodically during teleoperated mode. */
  /*
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }
  */

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    LiveWindow.setEnabled(false);
    if (!m_robotContainer.isLeadscrewInitialized()) c_InitializeLeadscrew.schedule();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}