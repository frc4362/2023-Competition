// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics.robot;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.commands.AutoBalanceCommand;
import com.gemsrobotics.robot.commands.TeleopSwerve;
import com.gemsrobotics.robot.subsystems.*;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private CommandBase m_autonomousCommand;
  private final XboxController m_joystickPilot = new XboxController(0);
  private final XboxController m_joystickCopilot = new XboxController(1);
  private final JoystickButton m_pickupButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kA.value);
  private final JoystickButton m_placementMidButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kB.value);
  private final JoystickButton m_placementHighButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kY.value);
  private final JoystickButton m_resetButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kX.value);
  //private final JoystickButton m_resetFieldOrientation = new JoystickButton(m_joystickCopilot, XboxController.Button.kY.value);

  private boolean m_selfPickup = false;

  private boolean m_dropPiece = false;
  private static final String PIVOT_KEY = "pivot_angle";
  private static final String ELEVATOR_KEY = "elevator_height";
  private static final String INTAKE_KEY = "intake_pos";
  private static final String WRIST_KEY = "wrist_angle_pos";
  double pivotRef = Double.NaN;

  private Command m_teleopSwerveCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CommandScheduler.getInstance().registerSubsystem(
            Claw.getInstance(),
            Intake.getInstance(),
            Wrist.getInstance(),
            Pivot.getInstance(),
            Elevator.getInstance(),
            ArmManager.getInstance(),
            Swerve.getInstance(),
            Superstructure.getInstance()
    );

    LEDController.getInstance().ifPresent(CommandScheduler.getInstance()::registerSubsystem);

    m_pickupButton.debounce(0.2, Debouncer.DebounceType.kRising);
    m_pickupButton.onTrue(new InstantCommand(
            () -> Superstructure.getInstance().setGoalPose(SuperstructurePose.SHELF_PICKUP)));

    m_placementMidButton.debounce(0.2, Debouncer.DebounceType.kRising);
    m_placementMidButton.onTrue(new InstantCommand(
            () -> Superstructure.getInstance().setGoalPose(SuperstructurePose.MID_PLACE)));

    m_placementHighButton.debounce(0.2, Debouncer.DebounceType.kRising);
    m_placementHighButton.onTrue(new InstantCommand(
            () -> Superstructure.getInstance().setGoalPose(SuperstructurePose.HIGH_PLACE)));

    m_resetButton.debounce(0.2, Debouncer.DebounceType.kRising);
    m_resetButton.onTrue(new InstantCommand(Superstructure.getInstance()::setGoalPoseCleared));

    m_teleopSwerveCommand = new TeleopSwerve(
            Swerve.getInstance(),
            () -> -m_joystickPilot.getLeftY(),
            () -> -m_joystickPilot.getLeftX(),
            () -> -m_joystickPilot.getRightX(),
            m_joystickPilot::getLeftBumper
    );

    m_autonomousCommand = new AutoBalanceCommand(Swerve.getInstance());

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
//    Elevator.getInstance().log();
    //Wrist.getInstance().log();
//    Claw.getInstance().log();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancel(m_teleopSwerveCommand);
  }

  @Override
  public void disabledPeriodic() {
  }

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
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().schedule(m_teleopSwerveCommand);
    LimelightHelpers.setAlliance(DriverStation.getAlliance());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_joystickPilot.getStartButtonPressed()) {
      Swerve.getInstance().zeroGyro();
    }
    Claw.getInstance().setDriveJoy(m_joystickPilot.getYButton()); 

//    if (m_joystick.getAButtonPressed()) {
//      m_selfPickup = !m_selfPickup;
//    }
//
//    if (m_selfPickup) {
//      Superstructure.getInstance().setTargetPose(SuperstructurePose.SHELF_PICKUP);
//    } else {
//      Superstructure.getInstance().setWantedState(Superstructure.WantedState.STOWED);
//    }

//    if (m_joystick.getAButton()) {
//      Superstructure.getInstance().setTargetPose(SuperstructureState.SHELF_PICKUP);
//    } else {
//      Superstructure.getInstance().setWantedState(Superstructure.WantedState.STOWED);
//    }

    if (m_joystickPilot.getRightBumperPressed()) {
      if (m_dropPiece) {
        Claw.getInstance().requestDropPiece().schedule();
      } else {
        Claw.getInstance().requestGrab().schedule();
      }

      m_dropPiece = !m_dropPiece;
    }

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
