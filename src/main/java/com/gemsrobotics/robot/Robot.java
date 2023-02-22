// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.subsystems.Elevator;
import com.gemsrobotics.robot.subsystems.Pivot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private XboxController m_controller = new XboxController(0);
  private Command m_autonomousCommand;
  //private Pivot m_pivot;
  private Elevator m_elevator;

  private static final String PIVOT_KEY = "pivot_angle";
  double pivotRef = Double.NaN;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    m_elevator = new Elevator();
    //m_pivot = new Pivot();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
   //  autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    CommandScheduler.getInstance().registerSubsystem(m_robotContainer.getSwerve());
    CommandScheduler.getInstance().registerSubsystem(m_elevator);
    //CommandScheduler.getInstance().registerSubsystem(m_pivot);
    SmartDashboard.putNumber(PIVOT_KEY, 90);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //m_pivot.disable();
    m_elevator.disable();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
//    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    LimelightHelpers.setAlliance(DriverStation.getAlliance());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    LimelightHelpers.setAlliance(DriverStation.getAlliance());
    //m_pivot.enable(); TODO PIVOT
    //m_elevator.enable(); TODO ELEVATOR
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // double newP = SmartDashboard.getNumber(PIVOT_KEY, Double.NaN); TODO PIVOT

    // if (!Double.isNaN(newP)) {
    //   m_pivot.setReference(Rotation2d.fromDegrees(newP));
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
