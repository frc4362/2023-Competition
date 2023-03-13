package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.SwerveModule;
import com.gemsrobotics.robot.Constants;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;
import java.util.Objects;

import static java.lang.Math.abs;

public final class Swerve implements Subsystem {
    private static Swerve INSTANCE;

    public static Swerve getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new Swerve();
        }

        return INSTANCE;
    }

    private static final double STARTING_ANGLE = 180;
    private static final int MOVING_AVERAGE_SIZE = 5;

    private final SwerveDrivePoseEstimator m_swervePoseEstimator;
    private final List<SwerveModule> m_swerveModules;
    private final Pigeon2 m_imu;
    private final Field2d m_fieldDisplay;
    private final LinearFilter m_deltaPitchFilter;
    private double m_deltaPitchAverage;
    private double m_lastPitch;

    public Swerve() {
        m_imu = new Pigeon2(Constants.Swerve.pigeonID);
        m_imu.configFactoryDefault();
        m_imu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 5);

        // old
        m_swerveModules = List.of(
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        );

//        m_swerveModules = List.of(
//            new SwerveModule(0, Constants.Swerve.Mod2.constants),
//            new SwerveModule(1, Constants.Swerve.Mod3.constants),
//            new SwerveModule(2, Constants.Swerve.Mod0.constants),
//            new SwerveModule(3, Constants.Swerve.Mod1.constants)
//        );

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        m_imu.setYaw(STARTING_ANGLE);

        m_deltaPitchFilter = LinearFilter.movingAverage(MOVING_AVERAGE_SIZE);
        m_lastPitch = m_imu.getRoll();
        m_deltaPitchAverage = 0.0;

        m_fieldDisplay = new Field2d();
        SmartDashboard.putData(m_fieldDisplay);

        // TODO tune the filter
        m_swervePoseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getYaw(),
                getModulePositions(),
//                new Pose2d(new Translation2d(3, 7), Rotation2d.fromDegrees(0)),
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(STARTING_ANGLE)),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10)));
    }

    public void setDrivePercent(
            final Translation2d translation,
            final double rotation,
            final boolean fieldRelative,
            final boolean isOpenLoop
    ) {
        setDrive(
                translation.times(Constants.Swerve.maxSpeed),
                rotation,
                fieldRelative,
                isOpenLoop
        );
    }

    public void setDrive(
            final Translation2d translation,
            final double rotation,
            final boolean fieldRelative,
            final boolean isOpenLoop
    ) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (final var mod : m_swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setNeutral() {
        setDrive(new Translation2d(), 0, false, true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(final SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for (final var mod : m_swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return m_swervePoseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(final Pose2d pose) {
        m_swervePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (final var mod : m_swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (final var mod : m_swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        m_imu.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_imu.getYaw()) : Rotation2d.fromDegrees(m_imu.getYaw());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(m_imu.getRoll());
    }

    public Rotation2d getDeltaPitch() {
        return Rotation2d.fromDegrees(m_deltaPitchAverage);
    }

    public Command waitForPitchAround(final Rotation2d targetPitch, final double tolerance) {
        return new WaitUntilCommand(() -> abs(getPitch().minus(targetPitch).getDegrees()) < tolerance);
    }

    public Command waitForPitchAround(final Rotation2d targetPitch) {
        return waitForPitchAround(targetPitch, 1.5);
    }

    public void resetModulesToAbsolute() {
        for (final var mod : m_swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public boolean isPivotAllowed() {
        boolean allowed = true;

        for (final var module : m_swerveModules) {
            allowed &= (Math.abs(module.getState().speedMetersPerSecond) < (Constants.Swerve.maxSpeed * 5));
        }

        return allowed;
    }

    public Command getTrackingCommand(final PathPlannerTrajectory trajectory, final boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if (isFirstPath) {
                    final var transformedState = PathPlannerTrajectory.transformStateForAlliance(
                            trajectory.getInitialState(),
                            DriverStation.getAlliance());
                    final var initialHolonomicPose = new Pose2d(
                            transformedState.poseMeters.getTranslation(),
                            transformedState.holonomicRotation);

                    resetOdometry(initialHolonomicPose);
                }
            }),
            new PPSwerveControllerCommand(
                trajectory,
                this::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                Constants.Generation.pureThetaController,
                this::setModuleStates,
                true, // Should we mirror path
                this
            )
        );
    }

    public Command getStopCommand() {
        return runOnce(this::setNeutral);
    }

    public Command getResetOdometryCommand(final Trajectory trajectory) {
        return runOnce(() -> resetOdometry(trajectory.getInitialPose()));
    }

    public void log() {
        for (final var mod : m_swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    @Override
    public void periodic() {
        // pitch filtering
        final var currentPitch = getPitch().getDegrees();
        final var deltaPitch = currentPitch - m_lastPitch;
        m_deltaPitchAverage = m_deltaPitchFilter.calculate(deltaPitch);
        m_lastPitch = getPitch().getDegrees();

        // pose estimation
        m_swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());

        if (Constants.Features.DO_VISION_FILTER) {
            LimelightHelpers.getBotpose_wpiAlliance("").map(Pose3d::toPose2d).ifPresent(measurement -> {
                if (m_swervePoseEstimator.getEstimatedPosition().getTranslation().getDistance(measurement.getTranslation()) < Constants.VISION_OUTLIER_DISTANCE) {
                    final var imageCaptureTime = LimelightHelpers.getLatency_Cl("") / 1_000.0;
                    final var imageProcessTime = LimelightHelpers.getLatency_Pipeline("") / 1_000.0;
                    m_swervePoseEstimator.addVisionMeasurement(
                            measurement,
                            Timer.getFPGATimestamp() - (imageCaptureTime + imageProcessTime));
                }
            });
        }

        // TODO does this even show up on the map at all lol
//        final var FIELD = new Translation2d(15.980, 8.21);
//        final var estimated = m_swervePoseEstimator.getEstimatedPosition();
//        final var fixedPose = FIELD.minus(estimated.getTranslation());
        m_fieldDisplay.setRobotPose(m_swervePoseEstimator.getEstimatedPosition());

        // SmartDashboard.putNumber("Yaw", m_imu.getYaw());
        // SmartDashboard.putNumber("Pitch", m_imu.getPitch());
        // SmartDashboard.putNumber("Roll", m_imu.getRoll());
        SmartDashboard.putString("Pose", m_swervePoseEstimator.getEstimatedPosition().toString());
    }
}
