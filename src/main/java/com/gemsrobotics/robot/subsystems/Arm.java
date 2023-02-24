package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Arm extends Subsystem {
	private static final int PIVOT_MOTOR_ID = 15;
	private static final String PIVOT_MOTOR_BUS = "aux";

	// limits calculated from stowed starting position
	private static final int
			PIVOT_FORWARD_SOFT_LIMIT = 60_000,
			PIVOT_REVERSE_SOFT_LIMIT = -365_000;

	private static final double
			kP = 4.4,
			kD = 0.07;
	private static final double
			MAX_VELOCITY_RADIANS_PER_SECOND = 3.0,
			MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 15.0;

	// at 0 position units, arm 54 degrees off of the horizontal
	private static final Rotation2d PIVOT_STARTING_ANGLE = Rotation2d.fromDegrees(90); // 54 degrees is the starting angle
	private static final double TOLERANCE_DEGREES = 0.1;

	private final MotorController<TalonFX> m_motorPivot;
	private final MotorController<TalonFX> m_motorElevator;
	private final MotorController<TalonFX> m_motorWrist;
	
	private final ProfiledPIDController m_controllerPivot;
	private final ProfiledPIDController m_controllerElevator;
	private final ProfiledPIDController m_controllerWrist;
	
	private final ArmFeedforward m_feedforwardPivot;

	private Rotation2d m_reference;

	public Arm() {
		// pivot initialization

		m_motorPivot = MotorControllerFactory.createDefaultTalonFX(PIVOT_MOTOR_ID, PIVOT_MOTOR_BUS);
		m_motorPivot.setInvertedOutput(false);
		m_motorPivot.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motorPivot.setGearingParameters(1.0 / 531.67, 1.0);

		m_controllerPivot = new ProfiledPIDController(
				kP,
				0.0,
				kD,
				new TrapezoidProfile.Constraints(
						MAX_VELOCITY_RADIANS_PER_SECOND,
						MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED
				)
		);
		
		m_feedforwardPivot = new ArmFeedforward(0.103, 0.27, 0.161, 0.002);

		setReference(STARTING_ANGLE);
	}

	public void setReference(final Rotation2d reference) {
		setGoal(reference.getRadians());
	}

	public Rotation2d getPosition() {
		return Rotation2d.fromRotations(m_motorPivot.getPositionRotations()).rotateBy(PIVOT_STARTING_ANGLE);
	}

	@Override
	public void periodic() {
	}

	@Override
	protected void useOutput(final double feedback, final TrapezoidProfile.State setpoint) {
		final var feedforward = m_feedforwardPivot.calculate(setpoint.position, setpoint.velocity);
		m_motor.setVoltage(feedback, feedforward);
	}

	@Override
	protected double getMeasurement() {
		return getPosition().getRadians();
	}
}
