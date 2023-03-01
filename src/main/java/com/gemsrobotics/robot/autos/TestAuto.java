package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

import static com.gemsrobotics.robot.Constants.Generation.getBackwardTrajectory;
import static com.gemsrobotics.robot.Constants.Generation.getForwardTrajectory;

public class TestAuto extends SequentialCommandGroup {
	public TestAuto(final Swerve swerve) {
		final var start = new Translation2d(0, 0);
		final var point1 = new Translation2d(Units.inchesToMeters(72), Units.inchesToMeters(72));
		final var point2 = new Translation2d(Units.inchesToMeters(-72), 0);

		final var trajectory1 = getForwardTrajectory(
				new Pose2d(start, Rotation2d.fromDegrees(0)),
				List.of(
						point1,
						start,
						point2
				),
				new Pose2d(start, Rotation2d.fromDegrees(180)));

		addCommands(
				swerve.getResetOdometryCommand(trajectory1),
				swerve.getAbsoluteTrackingCommand(trajectory1),
				swerve.getStopCommand()
		);
	}
}
