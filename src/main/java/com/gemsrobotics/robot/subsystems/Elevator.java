package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.lib.util.Units;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Elevator extends ProfiledPIDSubsystem {
	private static final int MOTOR_ID = 18;
	private static final String MOTOR_BUS = "rio";
	// limits calculated from stowed starting position
	private static final double
			FORWARD_SOFT_LIMIT = (0.69*2.0),
			REVERSE_SOFT_LIMIT = 0;

	private static final double
			kP = 4.4, //TODO
			kD = 0.07; //TODO
	private static final double
			MAX_VELOCITY_RADIANS_PER_SECOND = 3.0, //TODO
			MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 15.0; //TODO

	// at 0 position units, arm 54 degrees off of the horizontal
	private static final double STARTING_LENGTH = 0.0;
	private static final double TOLERANCE_METERS = 0.01;

	private final MotorController<TalonFX> m_motor;
	//private final PIDController m_controller;
	//private final ElevatorFeedforward m_feedforward;

	public Elevator() {
		super(
				new ProfiledPIDController(
						kP,
						0.0,
						kD,
						new TrapezoidProfile.Constraints(
								MAX_VELOCITY_RADIANS_PER_SECOND,
								MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED
						)
				)
		);

		m_motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_ID, MOTOR_BUS);
		m_motor.setInvertedOutput(false);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE); //TODO may move to coast for measurements

		m_motor.setGearingParameters(1.0/(3.0*3.0), Units.inches2Meters(4.0*2)/(2.0*Math.PI));

		//m_feedforward = new ElevatorFeedforward(0.103, 0.27, 0.161, 0.002);

		//setReference(STARTING_LENGTH);
	}

	public void setReference(final double referenceMeters) {
		//setGoal(referenceMeters);
	}

	public double getPosition() {
		return m_motor.getPositionMeters();
	}

	@Override
	public void periodic() {
		super.periodic();
		SmartDashboard.putNumber("Elevator Length", getPosition());
		SmartDashboard.putNumber("Elevator Drawn Amps", m_motor.getDrawnCurrentAmps());
		SmartDashboard.putNumber("Elevator Control Effort", m_motor.getVoltageOutput());
		SmartDashboard.putNumber("Elevator Velocity", m_motor.getVelocityLinearMetersPerSecond());
	}

	@Override
	protected void useOutput(final double feedback, final TrapezoidProfile.State setpoint) {
		//final var feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
		//m_motor.setVoltage(feedback, feedforward);
	}

	@Override
	protected double getMeasurement() {
		return getPosition();
	}
}
