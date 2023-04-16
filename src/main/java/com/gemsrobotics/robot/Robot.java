// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.autos.*;
import com.gemsrobotics.robot.commands.*;
import com.gemsrobotics.robot.subsystems.*;
import com.gemsrobotics.robot.subsystems.Intake.TargetHeight;
import com.gemsrobotics.robot.subsystems.LEDController.State;
import com.gemsrobotics.robot.subsystems.Superstructure.SystemState;
import com.gemsrobotics.robot.subsystems.Superstructure.WantedState;

import edu.wpi.first.math.MathUtil;
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
          m_hybridButton,
          m_testIntake;
  public POVButton
          m_shootHighButton,
          m_shootMidButton,
          m_shootHybridButton,
          m_shootExhaustButton,
          m_clearIntakeButton;

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

    // Copilot buttons <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
           () -> Intake.getInstance().setOuttakeType(Intake.TargetHeight.BOWLING)));

    // m_hatButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kLeftBumper.value);
  
    m_clawOpenButton = new JoystickButton(m_joystickCopilot, XboxController.Button.kRightBumper.value);
    m_clawOpenButton.debounce(1., Debouncer.DebounceType.kRising);
    m_clawOpenButton.onTrue(Claw.getInstance().requestDropPiece());

    //} // End copilot controls <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    
    // pilot controls <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    m_joystickPilot = new XboxController(Constants.PILOT_PORT);

    m_clearIntakeButton = new POVButton(m_joystickPilot, 0);
    m_clearIntakeButton.onTrue(Commands.runOnce(() -> Intake.getInstance().setOuttakeType(TargetHeight.CLEAR_INTAKE)));

    m_clawCloseButton = new JoystickButton(m_joystickPilot, XboxController.Button.kRightBumper.value);
    m_clawCloseButton.debounce(1., Debouncer.DebounceType.kRising);
    m_clawCloseButton.onTrue(Claw.getInstance().requestGrab());

    m_resetFieldOrientationButton = new JoystickButton(m_joystickPilot, XboxController.Button.kStart.value);
    m_resetFieldOrientationButton.debounce(Constants.DEBOUNCE_TIME_SECONDS, Debouncer.DebounceType.kRising);

    m_intakingButton = new JoystickButton(m_joystickPilot, XboxController.Button.kLeftBumper.value);
    final var intakeCommand = new IntakeUntilCubeCommand(Double.POSITIVE_INFINITY);
    m_intakingButton.onTrue(intakeCommand);
    m_intakingButton.onFalse(new InstantCommand(intakeCommand::cancel));

    m_pilotShootButton = new Trigger(() -> m_joystickPilot.getLeftTriggerAxis() > 0.9);
    m_pilotShootButton.onTrue(new InstantCommand(() -> Superstructure.getInstance().setWantedState(WantedState.OUTTAKING)));
    m_pilotShootButton.onFalse(new InstantCommand(() -> {
      Superstructure.getInstance().setWantedState(WantedState.STOWED);
      Intake.getInstance().setCubeOffsetCleared();
    }));
    
    m_teleopSwerveCommand = new TeleopSwerve(
            Swerve.getInstance(),
            () -> -m_joystickPilot.getLeftY(),
            () -> -m_joystickPilot.getLeftX(),
            () -> -m_joystickPilot.getRightX(),
            () -> false,
            () -> Pivot.getInstance().isInhibitingMobility() || m_joystickPilot.getRightStickButton(),
            Superstructure.getInstance()::doPickupSlow,
            m_joystickPilot::getRightStickButton,
            () -> Constants.Features.DO_EVASION && m_joystickPilot.getRightTriggerAxis() > 0.8,
            () -> {
              return m_joystickPilot.getLeftTriggerAxis() > 0.3 
                && (Intake.getInstance().getState() == Intake.State.OUTTAKING_MID || Intake.getInstance().getState() == Intake.State.OUTTAKING_HIGH);
            }
    );

    // End pilot controls <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    m_autonChooser = new SendableChooser<>();
    m_autonChooser.addOption("None", new WaitCommand(1.0));
    m_autonChooser.addOption("Auto balance auton", new BalanceAuto(Swerve.getInstance()));
    // m_autonChooser.addOption("Two.5 auton", new TwoAndBalanceAuto());
    m_autonChooser.addOption("Three and balance auton", new AutoTimedWheelLock(new ThreeAndBalanceAuto(), 15.2));
    m_autonChooser.addOption("Three and middle auton", new ThreeAndMidAuto());
    m_autonChooser.addOption("Cable auton", new ThreeCableAuto());
//    m_autonChooser.addOption("Straight auton", new DriveStraightAuton());
    m_autonChooser.addOption("Test balance auto", new FeedbackBalanceCommand(true));
//    m_autonChooser.addOption("Test placement auto", new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.STARTING))
//        .andThen(Claw.getInstance().requestGrab())
//        .andThen(new PlaceCommand(SuperstructurePose.AUTON_PLACE)));
    m_autonChooser.addOption("Test Cube Shoot auto",
           new CenterOnTagCommand(() -> 0.0, () -> new Translation2d(0.0, 0.0))
             .andThen(new WaitCommand(0.1))
             .andThen(new ShootCommand(Intake.TargetHeight.HIGH_AUTO, 0.5)));
    m_autonChooser.addOption("2.5 Auto", new TwoAndBalanceAuto());
    m_autonChooser.addOption("AprilTag localization test", Swerve.getInstance().getOdometryResetOnVisionCommand());

      // .andThen(new AttainPoseCommand(SuperstructurePose.MID_PLACE))
      // .andThen(new WaitUntilCommand(() -> Claw.getInstance().getObservedPiece().isPresent() && Claw.getInstance().getPieceConfidence()))
    SmartDashboard.putData(m_autonChooser);
    SmartDashboard.putString("Relocalized", "Not yet");
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
//    Intake.getInstance().log();
//    Pivot.getInstance().log();
//    Elevator.getInstance().log();
//     Wrist.getInstance().log();
    Claw.getInstance().log();
    Swerve.getInstance().log();
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

    LimelightHelpers.setLEDMode_ForceOff("");
  }

  /** This autonomous runs the autonomous command selected by your class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    LimelightHelpers.setAlliance(DriverStation.getAlliance());

    m_autonomousCommand = m_autonChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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
    LimelightHelpers.setLEDMode_PipelineControl("");
    LimelightHelpers.setPipelineIndex("", 1);

    Swerve.getInstance().setNeutralMode(NeutralMode.Brake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_joystickPilot.getRightStickButtonPressed()) {
      LimelightHelpers.setPipelineIndex("", 0);
    } else if (m_joystickPilot.getRightStickButtonReleased()) {
      LimelightHelpers.setPipelineIndex("", 1);
    }

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

    final var pivotScrub = MathUtil.applyDeadband(-m_joystickCopilot.getLeftY(), 0.8);
    if (pivotScrub != 0) {
      Pivot.getInstance().scrubSetpoint(pivotScrub * 10.0 / 50.0); // 1000ms / 20ms = 50.0
    }
  }
}
