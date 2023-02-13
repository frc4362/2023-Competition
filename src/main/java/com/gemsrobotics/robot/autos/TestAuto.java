package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestAuto extends SequentialCommandGroup {


    public TestAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        TrajectoryConfig configB =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setReversed(true)
                        .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory1 =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(3, 7), Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(new Translation2d(Units.inchesToMeters(65), Units.inchesToMeters(220.0)), Rotation2d.fromDegrees(-180)),
                config);

        Trajectory trajectory2 =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(new Translation2d(Units.inchesToMeters(65), Units.inchesToMeters(220.0)), Rotation2d.fromDegrees(-180)),
                        List.of(),
                        new Pose2d(new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(220.0)), Rotation2d.fromDegrees(-180)),
                        configB);

        Trajectory trajectory3 =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(220.0)), Rotation2d.fromDegrees(-180)),
                        List.of(),
                        new Pose2d(new Translation2d(3, 7), Rotation2d.fromDegrees(0.0)),
                        configB);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                trajectory1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand2 =
                new SwerveControllerCommand(
                        trajectory2,
                        s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController,
                        s_Swerve::setModuleStates,
                        s_Swerve);

        SwerveControllerCommand swerveControllerCommand3 =
                new SwerveControllerCommand(
                        trajectory3,
                        s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController,
                        s_Swerve::setModuleStates,
                        s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())),
            swerveControllerCommand1,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(), 0, false, true)),
            new WaitCommand(3.0),
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory2.getInitialPose())),
            swerveControllerCommand2,
            swerveControllerCommand3,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(), 0, false, true))
        );
    }
}