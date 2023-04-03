package com.gemsrobotics.robot.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.commands.DriveOntoPlatform;
import com.gemsrobotics.robot.subsystems.Swerve;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeAndMidAuto extends SequentialCommandGroup {
	public ThreeAndMidAuto() {
		final var path4 = PathPlanner.loadPath("ThreePieceToMidAuto", Constants.Generation.constraints);

		addCommands(
				new ThreeAuto(),
				new InstantCommand(() -> Swerve.getInstance().setNeutralMode(NeutralMode.Coast)),
				Swerve.getInstance().getTrackingCommand(path4, true),
				Swerve.getInstance().getStopCommand()
		);
	}
}
