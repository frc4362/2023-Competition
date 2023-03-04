package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Claw;
import com.gemsrobotics.robot.subsystems.SuperstructurePose;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BalanceAuton extends SequentialCommandGroup {
	public BalanceAuton(final Swerve swerve) {
		addCommands(
				Claw.getInstance().requestGrab(),
				new AttainPoseCommand(SuperstructurePose.HIGH_PLACE),
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
