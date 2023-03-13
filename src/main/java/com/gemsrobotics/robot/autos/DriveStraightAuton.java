package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.subsystems.Swerve;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveStraightAuton extends SequentialCommandGroup {
	public DriveStraightAuton() {
		final var path = PathPlanner.loadPath("Straight Path", Constants.Generation.constraints);

		addCommands(
				Swerve.getInstance().getTrackingCommand(path, true),
				Swerve.getInstance().getStopCommand()
		);
	}
}
