package com.gemsrobotics.robot.subsystems;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.subsystems.Claw.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Claw implements Subsystem {
	private static final boolean DO_LOGGING = false;

	private static Claw INSTANCE = null;

	private static final int CLAW_DRIVE_MOTOR_ID = 19;
	private static final String CLAW_DRIVE_MOTOR_BUS = Constants.CANBusses.MAIN;
	private static final int CLAW_GRIP_MOTOR_ID = 14;
	private static final String CLAW_GRIP_MOTOR_BUS = Constants.CANBusses.MAIN;

	private static final double TOLERANCE = 0.05;

	private static final double GEARING_MULTIPLIER = 1.0 / 10.5;

	private static final double GRIP_CURRENT_LIMIT = 10.0;
	private static final double GRIP_DRIVE_VOLTAGE = 2.0;
	private static final double GRIP_OPEN_POSITION = -0.24;//TODO

	/*We liked running -1 Volts at 50A */

	private static double spin_power;
	private static boolean spin;

	public static final State GRIPPING = null;

	private final MotorController<TalonFX> m_motorDrive;
	private final MotorController<TalonFX> m_motorGrip;


	private static State m_state;

	public static Claw getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Claw();
		}

		return INSTANCE;
	}

	public Claw() {
		m_motorDrive = MotorControllerFactory.createDefaultTalonFX(CLAW_DRIVE_MOTOR_ID, CLAW_DRIVE_MOTOR_BUS);
		m_motorDrive.setInvertedOutput(true);
		m_motorDrive.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_motorGrip = MotorControllerFactory.createDefaultTalonFX(CLAW_GRIP_MOTOR_ID, CLAW_GRIP_MOTOR_BUS);
		m_motorGrip.setInvertedOutput(true);
		m_motorGrip.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motorGrip.setGearingParameters(GEARING_MULTIPLIER, 1.0);
		m_motorGrip.setOpenLoopVoltageRampRate(0.6);
		//m_motorGrip.setCurrentLimit(50); does not work??

		spin_power = 0;
		spin = false;
		
		m_state = State.NEUTRAL;
	}

	public enum State {
		NEUTRAL,
		GRIPPING,
		OPEN
	}

	public void setReference(final State state) {
		m_state = state;
	}

	public void setSpin(boolean s) {
		spin = s;
	}

	public double getPosition() {
		return m_motorDrive.getPositionRotations();
	}

	//  public boolean atReference(final double toleranceRotations) {
	//  	return Math.abs(m_reference - m_motorDrive.getPositionRotations()) < toleranceRotations;
	//  }

	//  public boolean atReference() {
	// 	return atReference(TOLERANCE);
	//  }

	public void log() {
		SmartDashboard.putNumber("Claw Grip Rotations", m_motorGrip.getPositionRotations());
		SmartDashboard.putNumber("Claw Grip Drawn Amps", m_motorGrip.getDrawnCurrentAmps());
		SmartDashboard.putNumber("Claw Grip Control Effort", m_motorGrip.getVoltageOutput());
		SmartDashboard.putString("Claw State", m_state.toString());
		SmartDashboard.putBoolean("Claw Spin", spin);
	}

	public void setSpinPower(double grip) {
		spin_power = grip*6.0;
	}

	@Override
	public void periodic() {
		if(spin) {
			m_motorDrive.setVoltage(spin_power);
		} else {
			m_motorDrive.setVoltage(0.0);
		}
		switch(m_state) {
			case GRIPPING:
				m_motorGrip.setVoltage(-1.0);//-1.0
				break;
			case OPEN:
				if(m_motorGrip.getPositionRotations()<GRIP_OPEN_POSITION) {
					m_motorGrip.setVoltage(2.0);
				} else {
					m_motorGrip.setVoltage(0.0);
					m_state = Claw.State.NEUTRAL;
				}
				break;
			case NEUTRAL:
				m_motorGrip.setVoltage(0.0);
				break;
		}
	}
}
