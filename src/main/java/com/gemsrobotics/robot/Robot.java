// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics.robot;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.commands.TeleopSwerve;
import com.gemsrobotics.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private CommandBase m_autonomousCommand;
  private XboxController m_joystick = new XboxController(0);
  private static final String PIVOT_KEY = "pivot_angle";
  private static final String ELEVATOR_KEY = "elevator_height";
  private static final String INTAKE_KEY = "intake_pos";
  private static final String WRIST_KEY = "wrist_angle_pos";
  double pivotRef = Double.NaN;

  private Command m_teleopSwerveCommand;

  private Claw m_claw;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_claw = new Claw();
    CommandScheduler.getInstance().registerSubsystem(
            Intake.getInstance(),
            Wrist.getInstance(),
            Pivot.getInstance(),
            Elevator.getInstance(),
            ArmManager.getInstance(),
            Swerve.getInstance(),
            Claw.getInstance(),
            Superstructure.getInstance()
    );

    m_teleopSwerveCommand = new TeleopSwerve(
            Swerve.getInstance(),
            () -> -m_joystick.getLeftY(),
            () -> -m_joystick.getLeftX(),
            () -> -m_joystick.getRightX(),
            () -> false
    );

    SmartDashboard.putNumber(PIVOT_KEY, Pivot.Position.STARTING.rotation.getDegrees());
    SmartDashboard.putNumber(ELEVATOR_KEY, 0.0);
    SmartDashboard.putNumber(INTAKE_KEY, 0.0);
    SmartDashboard.putNumber(WRIST_KEY, Wrist.STARTING_ANGLE_FROM_ELEVATOR.getDegrees());
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
    //Elevator.getInstance().log();
    //Wrist.getInstance().log();
    Claw.getInstance().log();
    SmartDashboard.putBoolean("Grip LeftBumper", m_joystick.getLeftBumper());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancel(m_teleopSwerveCommand);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your class. */
  @Override
  public void autonomousInit() {
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

//    CommandScheduler.getInstance().schedule(m_teleopSwerveCommand);
    LimelightHelpers.setAlliance(DriverStation.getAlliance());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_joystick.getAButton()) {
      //Superstructure.getInstance().openClaw();
      //Superstructure.getInstance().setWantScore(Superstructure.ScoringGoal.MID);
    } else {
      //Superstructure.getInstance().closeClaw();
      Superstructure.getInstance().setWantedState(Superstructure.WantedState.STOWED);
    }
    if(m_joystick.getLeftBumper()) {
      m_claw.setReference(Claw.State.GRIPPING);
    } else {
      m_claw.setReference(Claw.State.NEUTRAL);
    }
    // if(m_joystick.getRightBumper()) {
    //   m_claw.setReference(Claw.State.OPEN);
    // }
    m_claw.setSpinPower(m_joystick.getRightTriggerAxis());
    m_claw.setSpin(m_joystick.getXButton());

//    double newP = SmartDashboard.getNumber(PIVOT_KEY, Double.NaN); // TODO PIVOT
//
//    if (!Double.isNaN(newP)) {
//      Pivot.getInstance().setReference(Rotation2d.fromDegrees(newP));
//    }

//     Pivot.getInstance().setReference(Rotation2d.fromDegrees(54));
//     Elevator.getInstance().setReference(m_joystick.getAButton() ? Elevator.Setpoints.TEST_POSITION : Elevator.Setpoints.SAFETY_BOTTOM);

//    Elevator.getInstance().setReference(MathUtils.coerce(0.02, SmartDashboard.getNumber(ELEVATOR_KEY, 0.02), 1.38));
////    Elevator.getInstance().setOpenLoop(SmartDashboard.getNumber(ELEVATOR_KEY, 0.0));
//    Intake.getInstance().setReference(SmartDashboard.getNumber(INTAKE_KEY, 0.0));
//    Wrist.getInstance().setReferenceAngleToElevator(Units.degreesToRotations(MathUtils.coerce(
//            -54,
//            SmartDashboard.getNumber(WRIST_KEY, Wrist.STARTING_ANGLE_FROM_ELEVATOR.getDegrees()),
//            160)));
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
