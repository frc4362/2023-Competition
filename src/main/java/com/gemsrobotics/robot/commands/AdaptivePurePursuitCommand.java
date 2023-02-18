package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class AdaptivePurePursuitCommand {
	private final Trajectory m_trajectory;
	private final Supplier<Pose2d> m_robotPose;
	private final SwerveDriveKinematics m_kinematics;

	public AdaptivePurePursuitCommand(
			final Trajectory trajectory,
			final Supplier<Pose2d> robotPose,
			final SwerveDriveKinematics kinematics,
			final PIDController controllerX,
			final PIDController controllerY,
			final ProfiledPIDController controllerTheta,
			final Consumer<SwerveModuleState[]> output,
			final Swerve swerve
	) {
		m_trajectory = trajectory;
		m_robotPose = robotPose;
		m_kinematics = kinematics;

	}
}
