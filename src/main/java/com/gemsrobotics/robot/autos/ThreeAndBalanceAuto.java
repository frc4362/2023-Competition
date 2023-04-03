package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.commands.DriveOntoPlatform;
import com.gemsrobotics.robot.commands.ShootCommand;
import com.gemsrobotics.robot.subsystems.Intake;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreeAndBalanceAuto extends SequentialCommandGroup {
	public ThreeAndBalanceAuto() {
		addCommands(
				new ThreeAuto(),
				new ParallelCommandGroup(
						new DriveOntoPlatform(Swerve.getInstance(), new Translation2d(1.00, 0.0), 0.35)
				),
				new InstantCommand(() -> Swerve.getInstance().setDrivePercent(new Translation2d(0, 0), 0.25, true, false)),
				new WaitCommand(0.25),
				Swerve.getInstance().getStopCommand()
		);
	}
}
