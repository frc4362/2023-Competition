package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.commands.SuperstructurePoseCommand;
import com.gemsrobotics.robot.subsystems.Claw;
import com.gemsrobotics.robot.subsystems.SuperstructurePose;
import com.gemsrobotics.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class SideAuto extends SequentialCommandGroup {
    public SideAuto() {
        final var trajectory = Constants.Generation.getBackwardTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
            new Pose2d(Units.feetToMeters(15), 0, Rotation2d.fromDegrees(0))
        );

        addCommands(
            Claw.getInstance().requestGrab(),
            new SuperstructurePoseCommand(SuperstructurePose.HIGH_PLACE),
            Swerve.getInstance().getAbsoluteTrackingCommand(trajectory)
        );
    }
}
