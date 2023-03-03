package com.gemsrobotics.robot.commands;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public class LimelightDriveCommand implements Command {
	private final static double kP = 0.1;

	private final Swerve m_swerve;

	public LimelightDriveCommand(final Swerve swerve) {
		m_swerve = swerve;
	}

	@Override
	public void execute() {
		final double error = LimelightHelpers.getTX("");

		m_swerve.setDrivePercent(
				new Translation2d(0, kP * error),
				0.0,
				false,
				true
		);
	}

	@Override
	public Set<Subsystem> getRequirements() {
		return Set.of(m_swerve);
	}
}
