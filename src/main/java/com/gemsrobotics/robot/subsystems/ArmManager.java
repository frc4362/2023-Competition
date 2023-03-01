package com.gemsrobotics.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

public final class ArmManager implements Subsystem {
	private static ArmManager INSTANCE;

	public static ArmManager getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new ArmManager();
		}

		return INSTANCE;
	}

	private final Pivot m_pivot;
	private final Elevator m_elevator;
	private final Wrist m_wrist;

	private ArmManager() {
		m_pivot = Pivot.getInstance();
		m_elevator = Elevator.getInstance();
		m_wrist = Wrist.getInstance();
	}

	@Override
	public void periodic() {
		m_elevator.setExternalAngle(m_pivot.getAngle());
		m_wrist.setExternalAngle(m_pivot.getAngle());
//		m_pivot.setReference(m_reference.rotationPivot);
//		m_elevator.setReference(m_reference.elevatatorExtension);
//		m_wrist.setReferenceAngleToElevator(m_reference.rotationWrist);
	}
}
