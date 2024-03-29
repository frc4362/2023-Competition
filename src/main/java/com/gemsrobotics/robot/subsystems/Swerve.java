package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.util.Translation2dPlus;
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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

public final class Swerve implements Subsystem {
    private static final Translation2d[] WHEEL_POSITIONS =
            Arrays.copyOf(Constants.Swerve.moduleTranslations, Constants.Swerve.moduleTranslations.length);;

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

    private Rotation2d m_gyroOffset;

    public Swerve() {
        m_imu = new Pigeon2(Constants.Swerve.pigeonID);
        m_imu.configFactoryDefault();
        m_imu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 5);

        m_gyroOffset = Rotation2d.fromDegrees(0);

        // old
        m_swerveModules = List.of(
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        );

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
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.1));
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
            final boolean isOpenLoop,
            final boolean isEvading
    ) {
        final Translation2d centerOfRotation;

        if (isEvading && fieldRelative) {
            centerOfRotation = getCenterOfRotation(translation.getAngle(), rotation);
        } else {
            centerOfRotation = new Translation2d();
        }

        final ChassisSpeeds chassisSpeeds;

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getYaw()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
            );
        }

        SmartDashboard.putString("CoR", centerOfRotation.toString());

        final var swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (final var mod : m_swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setDrive(
            final Translation2d translation,
            final double rotation,
            final boolean fieldRelative,
            final boolean isOpenLoop
    ) {
        setDrive(translation, rotation, fieldRelative, isOpenLoop, false);
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

    private Translation2d getCenterOfRotation(final Rotation2d direction, final double rotation) {
        final var here = new Translation2dPlus(1.0, direction.minus(getYaw()));

        var cwCenter = WHEEL_POSITIONS[0];
        var ccwCenter = WHEEL_POSITIONS[WHEEL_POSITIONS.length - 1];

        for (int i = 0; i < WHEEL_POSITIONS.length - 1; i++) {
            final var cw = WHEEL_POSITIONS[i];
            final var ccw = WHEEL_POSITIONS[i + 1];

            if (here.isWithinAngle(cw, ccw)) {
                cwCenter = ccw;
                ccwCenter = cw;
            }
        }

        // if clockwise
        if (signum(rotation) == 1.0) {
            return cwCenter;
        } else if (signum(rotation) == -1.0) {
            return ccwCenter;
        } else {
            return new Translation2d();
        }
    }

    public void zeroGyro() {
//        m_imu.setYaw(0);

        //artificial zero
        final var baseYaw = (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_imu.getYaw()) : Rotation2d.fromDegrees(m_imu.getYaw());
        m_gyroOffset = baseYaw;
    }

    public Rotation2d getYaw() {
        final var baseYaw = (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_imu.getYaw()) : Rotation2d.fromDegrees(m_imu.getYaw());
        return baseYaw.minus(m_gyroOffset);
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(m_imu.getRoll());
    }

    public Rotation2d getDeltaPitch() {
        return Rotation2d.fromDegrees(m_deltaPitchAverage);
    }

    public Command waitForPitchAround(final Rotation2d targetPitch, final double tolerance) {
        return new WaitUntilCommand(() -> (abs(getPitch().minus(targetPitch).getDegrees())) < tolerance);
    }

    public Command waitForPitchAround(final Rotation2d targetPitch) {
        return waitForPitchAround(targetPitch, 1.5);
    }

    public void resetModulesToAbsolute() {
        for (final var mod : m_swerveModules) {
            mod.resetToAbsolute();
        }
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

    public void setNeutralMode(final NeutralMode neutral) {
        for (final var mod : m_swerveModules) {
            mod.setNeutral(neutral);
        }
    }

    
    public void setWheelLock() {
        
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        }
    );
    }   

    //     setModuleStates(new SwerveModuleState[] {
    //             new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    //             new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
    //             new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
    //             new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    //         }
    //     );   
    // }

    public void log() {
        for (final var mod : m_swerveModules) {
//            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
//            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public Command getOdometryResetOnVisionCommand() {
        return runOnce(() -> {
//            final var result = LimelightHelpers.getLatestResults("").targetingResults;

            LimelightHelpers.getBotpose_wpiAlliance("").map(Pose3d::toPose2d).ifPresentOrElse(measurement -> {
//                if (m_swervePoseEstimator.getEstimatedPosition().getTranslation().getDistance(measurement.getTranslation()) < Constants.VISION_OUTLIER_DISTANCE) {
                    final var imageCaptureTime = LimelightHelpers.getLatency_Cl("");
                    final var imageProcessTime = LimelightHelpers.getLatency_Pipeline("");
                    m_swervePoseEstimator.resetPosition(getYaw(), getModulePositions(), measurement);
//                    m_swervePoseEstimator.addVisionMeasurement(
//                            measurement,
//                            Timer.getFPGATimestamp() - ((imageCaptureTime + imageProcessTime) / 1_000));
                    SmartDashboard.putString("Relocalized", "Success");
//                }
            }, () -> {
                SmartDashboard.putString("Relocalized", "No measure");
            });
        });
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

//        // TODO does this even show up on the map at all lol
//        m_fieldDisplay.setRobotPose(m_swervePoseEstimator.getEstimatedPosition());

        SmartDashboard.putNumber("Yaw", m_imu.getYaw());
        SmartDashboard.putNumber("Pitch", m_imu.getPitch());
        SmartDashboard.putNumber("Roll", m_imu.getRoll());
        SmartDashboard.putString("Pose", m_swervePoseEstimator.getEstimatedPosition().toString());
    }
}
