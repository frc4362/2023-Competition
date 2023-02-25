package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.lib.util.Units;

import com.gemsrobotics.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

public final class Elevator implements Subsystem {
	private static final boolean DO_LOGGING = false;

	private static Elevator INSTANCE = null;

	public static Elevator getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Elevator();
		}

		return INSTANCE;
	}

	private static final int MOTOR_ID = 18;
	private static final String MOTOR_BUS = Constants.CANBusses.MAIN;

	// limits calculated from starting position at the bottom
	private static final double
			FORWARD_SOFT_LIMIT = 100_000,
			REVERSE_SOFT_LIMIT = 0;

	private static final double STARTING_LENGTH = 0.0;
	private static final double TOLERANCE_METERS = 0.01;
	public static final double GEARING_MULTIPLIER = 1.0 / 9.0;

	public static final double
		kS = 0.0,
		kG = 0.0;

	private final MotorController<TalonFX> m_motor;
	private final PIDController m_controller;

	private double m_reference = 0.0;

	private Elevator() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_ID, MOTOR_BUS);
		m_motor.setInvertedOutput(false);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		// TODO
//		m_motor.setOpenLoopVoltageRampRate(0.0);

		m_motor.getInternalController().configForwardSoftLimitEnable(false);
		m_motor.getInternalController().configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
		m_motor.getInternalController().configReverseSoftLimitEnable(false);
		m_motor.getInternalController().configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);

		m_motor.setGearingParameters(GEARING_MULTIPLIER, Units.inches2Meters(4.0*2)/(2.0*Math.PI));

		m_controller = new PIDController(0.0, 0.0, 0.0);
	}

	public void setReference(final double referenceMeters) {
		//setGoal(referenceMeters);
	}

	public double getPosition() {
		return m_motor.getPositionMeters();
	}

	@Override
	public void periodic() {
		m_motor.setVoltage(m_controller.calculate(m_motor.getPositionMeters()), kG);

		if (DO_LOGGING) {
			SmartDashboard.putNumber("Elevator Length", getPosition());
			SmartDashboard.putNumber("Elevator Drawn Amps", m_motor.getDrawnCurrentAmps());
			SmartDashboard.putNumber("Elevator Control Effort", m_motor.getVoltageOutput());
			SmartDashboard.putNumber("Elevator Velocity", m_motor.getVelocityLinearMetersPerSecond());
		}
	}
}
