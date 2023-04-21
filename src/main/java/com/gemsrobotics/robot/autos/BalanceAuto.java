package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.commands.PlaceCommand;
import com.gemsrobotics.robot.commands.DriveOntoPlatform;
import com.gemsrobotics.robot.subsystems.Claw;
import com.gemsrobotics.robot.subsystems.Superstructure;
import com.gemsrobotics.robot.subsystems.SuperstructurePose;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BalanceAuto extends SequentialCommandGroup {
	public BalanceAuto(final Swerve swerve) {
		addCommands(
				new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.STARTING)),
				Claw.getInstance().requestGrab(), 
				new PlaceCommand(SuperstructurePose.AUTON_PLACE),
				new DriveOntoPlatform(swerve, new Translation2d(.35, 0.0), .20),
				new InstantCommand(() -> swerve.setDrivePercent(new Translation2d(.25, 0.0), 0.0, true, true)),
				new WaitCommand(1.),
				swerve.getStopCommand(),
				new WaitCommand(0.75),
				new DriveOntoPlatform(swerve, new Translation2d(-.35, 0.0), .15),
				// new RunCommand(() -> Swerve.getInstance().setWheelLock())
				new InstantCommand(() -> swerve.setDrivePercent(new Translation2d(0, 0), 0.25, true, true)),
				new WaitCommand(0.25),
				swerve.getStopCommand()
		);																																																																																																																																								
	}
}
