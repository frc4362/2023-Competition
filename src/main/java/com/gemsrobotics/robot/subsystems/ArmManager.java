package com.gemsrobotics.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

public class ArmManager implements Subsystem {
	private static ArmManager INSTANCE;

	public static ArmManager getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new ArmManager();
		}

		return INSTANCE;
	}

	private Pivot m_pivot;
	private Elevator m_elevator;

	private SuperstructureState m_reference;

	private ArmManager() {
		m_pivot = Pivot.getInstance();
		m_elevator = Elevator.getInstance();

		m_reference = SuperstructureState.STARTING;
	}

	public void setReference(final SuperstructureState reference) {
		m_reference = reference;
	}

	public boolean atReference() {
		return m_pivot.atReference();
	}

	@Override
	public void periodic() {
		m_pivot.setReference(m_reference.rotationPivot);
		m_elevator.setReference(m_reference.elevatatorExtension);
	}
}
