// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics.robot;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.autos.*;
import com.gemsrobotics.robot.commands.DriveOntoPlatform;
import com.gemsrobotics.robot.commands.PlaceCommand;
import com.gemsrobotics.robot.commands.TeleopSwerve;
import com.gemsrobotics.robot.subsystems.*;
import com.gemsrobotics.robot.subsystems.LEDController.State;
import com.gemsrobotics.robot.subsystems.Superstructure.SystemState;
import com.gemsrobotics.robot.subsystems.Superstructure.WantedState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public final class Robot extends TimedRobot {
  private Superstructure m_superstructure;
  private XboxController m_joystickPilot, m_joystickCopilot;
  private JoystickButton
          m_pickupButton,
          m_placementMidButton,
          m_placementHighButton,
          m_resetPoseButton,
          m_clawOpenButton,
          m_clawCloseButton,
          m_resetFieldOrientationButton,
          m_intakingButton,
          m_hybridButton;
  public POVButton
          m_shootHighButton,
          m_shootMidButton,
          m_shootHybridButton,
          m_shootExhaustButton;

  private SendableChooser<Command> m_autonChooser;
  private Command m_teleopSwerveCommand, m_autonomousCommand;

  private static final String PIVOT_KEY = "pivot_angle";
  private static final String ELEVATOR_KEY = "elevator_height";
  private static final String INTAKE_KEY = "intake_pos";
  private static final String WRIST_KEY = "wrist_angle_pos";

  private Trigger m_pilotShootButton;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CommandScheduler.getInstance().registerSubsystem(Claw.getInstance());
    CommandScheduler.getInstance().registerSubsystem(Intake.getInstance());
    CommandScheduler.getInstance().registerSubsystem(Wrist.getInstance());
    CommandScheduler.getInstance().registerSubsystem(Pivot.getInstance());
    CommandScheduler.getInstance().registerSubsystem(Elevator.getInstance());
    CommandScheduler.getInstance().registerSubsystem(ArmManager.getInstance());
    CommandScheduler.getInstance().registerSubsystem(Swerve.getInstance());
    CommandScheduler.getInstance().registerSubsystem(Superstructure.getInstance());

    LEDController.getInstance().ifPresent(CommandScheduler.getInstance()::registerSubsystem);

    m_superstructure = Superstructure.getInstance();

    // Copilot buttons
    m_joystickCopilot = new XboxController(Constants.COPILOT_PORT);

    m_pickupButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kA.value);
    m_pickupButton.debounce(Constants.DEBOUNCE_TIME_SECONDS, Debouncer.DebounceType.kRising);
    m_pickupButton.onTrue(Commands.runOnce(() -> {
      if (Superstructure.getInstance().getSystemState() == SystemState.STOWED) {
        m_superstructure.setGoalPose(SuperstructurePose.SHELF_PICKUP);
      }
    }, m_superstructure));

    m_placementMidButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kB.value);
    m_placementMidButton.debounce(Constants.DEBOUNCE_TIME_SECONDS, Debouncer.DebounceType.kRising);
    m_placementMidButton.onTrue(Commands.runOnce(() -> {
      if (Superstructure.getInstance().getSystemState() == SystemState.STOWED) {
        m_superstructure.setGoalPose(SuperstructurePose.MID_PLACE);
      }
    }, m_superstructure));

    m_placementHighButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kY.value);
    m_placementHighButton.debounce(Constants.DEBOUNCE_TIME_SECONDS, Debouncer.DebounceType.kRising);
    m_placementHighButton.onTrue(Commands.runOnce(() -> {
      if (Superstructure.getInstance().getSystemState() == SystemState.STOWED) {
        m_superstructure.setGoalPose(SuperstructurePose.HIGH_PLACE);
      }
    }, m_superstructure));

    m_resetPoseButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kStart.value);
    m_resetPoseButton.debounce(Constants.DEBOUNCE_TIME_SECONDS, Debouncer.DebounceType.kRising);
    m_resetPoseButton.onTrue(Commands.runOnce(m_superstructure::setGoalPoseCleared, m_superstructure));
    
    m_hybridButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kX.value);
    m_hybridButton.onTrue(new InstantCommand(() -> Superstructure.getInstance().setWantedState(WantedState.HYBRID)));
    m_hybridButton.onFalse(new InstantCommand(() -> Superstructure.getInstance().setWantedState(WantedState.STOWED)));

    m_shootHighButton = new POVButton(m_joystickCopilot, 0);
    m_shootHighButton.onTrue(Commands.runOnce(
            () -> Intake.getInstance().setOuttakeType(Intake.TargetHeight.HIGH)));

    m_shootMidButton = new POVButton(m_joystickCopilot, 90);
    m_shootMidButton.onTrue(Commands.runOnce(
           () -> Intake.getInstance().setOuttakeType(Intake.TargetHeight.MID)));

    m_shootHybridButton = new POVButton(m_joystickCopilot, 180);
    m_shootHybridButton.onTrue(Commands.runOnce(
           () -> Intake.getInstance().setOuttakeType(Intake.TargetHeight.HYBRID)));

    m_shootExhaustButton = new POVButton(m_joystickCopilot, 270);
    m_shootExhaustButton.onTrue(Commands.runOnce(
           () -> Intake.getInstance().setOuttakeType(Intake.TargetHeight.CLEAR_INTAKE)));

    // m_hatButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kLeftBumper.value);

    m_clawOpenButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kRightBumper.value);
    m_clawOpenButton.debounce(1., Debouncer.DebounceType.kRising);
    m_clawOpenButton.onTrue(Claw.getInstance().requestDropPiece());

    // pilot controls
    m_joystickPilot = new XboxController(Constants.PILOT_PORT);

    m_clawCloseButton = new JoystickButton(m_joystickPilot, XboxController.Button.kRightBumper.value);
    m_clawCloseButton.debounce(1., Debouncer.DebounceType.kRising);
    m_clawCloseButton.onTrue(Claw.getInstance().requestGrab());

    m_resetFieldOrientationButton = new JoystickButton(m_joystickPilot, XboxController.Button.kStart.value);
    m_resetFieldOrientationButton.debounce(Constants.DEBOUNCE_TIME_SECONDS, Debouncer.DebounceType.kRising);

    m_intakingButton = new JoystickButton(m_joystickPilot, XboxController.Button.kLeftBumper.value);
    final var intakeCommand = new InstantCommand(() -> Superstructure.getInstance().setWantedState(WantedState.INTAKING))
      .andThen(new WaitUntilCommand(Intake.getInstance()::isBeamBroken))
      .finallyDo(interrupted -> Superstructure.getInstance().setWantedState(WantedState.STOWED));
    m_intakingButton.onTrue(intakeCommand);
    m_intakingButton.onFalse(new InstantCommand(intakeCommand::cancel));

    m_pilotShootButton = new Trigger(() -> m_joystickPilot.getLeftTriggerAxis() > 0.7);
    m_pilotShootButton.onTrue(new InstantCommand(() -> Superstructure.getInstance().setWantedState(WantedState.OUTTAKING)));
    m_pilotShootButton.onFalse(new InstantCommand(() -> Superstructure.getInstance().setWantedState(WantedState.STOWED)));
    
    m_teleopSwerveCommand = new TeleopSwerve(
            Swerve.getInstance(),
            () -> -m_joystickPilot.getLeftY(),
            () -> -m_joystickPilot.getLeftX(),
            () -> -m_joystickPilot.getRightX(),
            m_joystickPilot::getBButton,
            Pivot.getInstance()::isInhibitingMobility,
            m_superstructure::isShelfPickupLimiter,
            () -> false
    );

    m_autonChooser = new SendableChooser<>();
    m_autonChooser.addOption("None", new WaitCommand(1.0));
    m_autonChooser.addOption("Auto balance auton", new BalanceAuto(Swerve.getInstance()));
    m_autonChooser.addOption("Two.5 auton", new TwoAndBalanceAuto());
    m_autonChooser.addOption("Three auton", new ThreeAuto());
    m_autonChooser.addOption("Straight auton", new DriveStraightAuton());
    m_autonChooser.addOption("Test auto", new DriveOntoPlatform(Swerve.getInstance(), new Translation2d(-.35, 0.0), .15)
        .andThen(Swerve.getInstance().getStopCommand()));
    m_autonChooser.addOption("Test placement auto", new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.STARTING))
        .andThen(Claw.getInstance().requestGrab())
        .andThen(new PlaceCommand(SuperstructurePose.AUTON_PLACE)));

      // .andThen(new AttainPoseCommand(SuperstructurePose.MID_PLACE))
      // .andThen(new WaitUntilCommand(() -> Claw.getInstance().getObservedPiece().isPresent() && Claw.getInstance().getPieceConfidence()))
    SmartDashboard.putData(m_autonChooser);

//    SmartDashboard.putNumber(PIVOT_KEY, Pivot.Position.STARTING.rotation.getDegrees());
//    SmartDashboard.putNumber(ELEVATOR_KEY, 0.0);
//    SmartDashboard.putNumber(INTAKE_KEY, 0.0);
//    SmartDashboard.putNumber(WRIST_KEY, Wrist.STARTING_ANGLE_FROM_ELEVATOR.getDegrees());
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
    // Intake.getInstance().log();
//    Pivot.getInstance().log();
//    Elevator.getInstance().log();
    // Wrist.getInstance().log();
//    Claw.getInstance().log();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_teleopSwerveCommand.cancel();
  }

  @Override
  public void disabledPeriodic() {
    if (m_autonChooser.getSelected() != null && !(m_autonChooser.getSelected() instanceof WaitCommand)) {
      LEDController.getInstance().ifPresent(controller -> controller.setState(State.IDLE));
    } else {
      LEDController.getInstance().ifPresent(controller -> controller.setState(State.OFF));
    }
  }

  /** This autonomous runs the autonomous command selected by your class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    m_autonomousCommand = m_autonChooser.getSelected();
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

    m_teleopSwerveCommand.schedule();
    LimelightHelpers.setAlliance(DriverStation.getAlliance());
    LimelightHelpers.setLEDMode_ForceOff("");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_joystickPilot.getStartButtonPressed()) {
      Swerve.getInstance().zeroGyro();
    }

    Superstructure.getInstance().setDoHat(m_joystickCopilot.getLeftBumper());

    if (m_joystickCopilot.getLeftTriggerAxis() > 0.9) {
      LEDController.getInstance().ifPresent(controller -> controller.setState(State.WANTS_CUBE));
    } else if (m_joystickCopilot.getRightTriggerAxis() > 0.9) {
      LEDController.getInstance().ifPresent(controller -> controller.setState(State.WANTS_CONE));
    } else if (m_joystickCopilot.getBackButton()) {
      LEDController.getInstance().ifPresent(controller -> controller.setState(State.WANTS_SHELF_CUBE));
    }
  }
}
