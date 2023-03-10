package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.commands.PlaceCommand;
import com.gemsrobotics.robot.commands.DriveOntoPlatform;
import com.gemsrobotics.robot.subsystems.Claw;
import com.gemsrobotics.robot.subsystems.SuperstructurePose;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BalanceAuto extends SequentialCommandGroup {
	public BalanceAuto(final Swerve swerve) {
		addCommands(
				Claw.getInstance().requestGrab(),
				new PlaceCommand(SuperstructurePose.HIGH_PLACE),
				new DriveOntoPlatform(swerve, new Translation2d(.35, 0.0), .35),
				new InstantCommand(() -> swerve.setDrivePercent(new Translation2d(.32, 0.0), 0.0, true, false)),
				new WaitCommand(.8),
				new DriveOntoPlatform(swerve, new Translation2d(-.35, 0.0), .125),
				new InstantCommand(() -> swerve.setDrivePercent(new Translation2d(0, 0), 0.25, true, false)),
				new WaitCommand(0.25),
				swerve.getStopCommand()
		);
	}
}
