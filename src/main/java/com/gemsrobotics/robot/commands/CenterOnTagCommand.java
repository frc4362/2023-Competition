package com.gemsrobotics.robot.commands;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.lib.util.Units;
import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.subsystems.Intake;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Supplier;

import static java.lang.Math.abs;

public class CenterOnTagCommand extends SequentialCommandGroup {
	private final static double TOLERANCE = 3; // degrees
	// meters per second per degree
	private final static double kP = 1.0 / 450.0;

	private final Supplier<Double> m_offset;

	public CenterOnTagCommand(final Supplier<Double> offsetSup, final Supplier<Translation2d> initialVelocity) {
		addRequirements(Swerve.getInstance());
		m_offset = offsetSup;
		addCommands(
				new InstantCommand(() -> LimelightHelpers.setPipelineIndex("", 1)),
				new RunCommand(() -> Swerve.getInstance().setDrivePercent(initialVelocity.get(), 0.0, true, true))
					.until(() -> LimelightHelpers.getCurrentPipelineIndex("") == 1 && LimelightHelpers.getTX("") != 0.0),
				new RunCommand(() -> {
					final double visionFeedback = kP * getError() * Constants.Swerve.maxSpeed;
					final double yawFeedback = Constants.Generation.pureThetaController.calculate(
							Swerve.getInstance().getYaw().getRadians(),
							Units.degrees2Rads(180)
					);

					final double depthFeedback = 4.5 * ((LimelightHelpers.getTA("")/100.0) - 0.055);

					Swerve.getInstance().setDrivePercent(new Translation2d(depthFeedback, visionFeedback), yawFeedback, true, true);
				}).until(() -> abs(getError()) < TOLERANCE)
				.andThen(Swerve.getInstance().getStopCommand())
		);
	}

	public static CenterOnTagCommand withIntakeRecordedOffset(final Supplier<Translation2d> initialVelocity) {
		return new CenterOnTagCommand(() -> Intake.getInstance().getCubeOffset().orElse(0.0), initialVelocity);
	}

	private double getError() {
		return LimelightHelpers.getTX("") - m_offset.get();
	}
}
