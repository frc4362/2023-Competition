package com.gemsrobotics.robot.subsystems;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.SwerveModule;
import com.gemsrobotics.robot.Constants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator m_swervePoseEstimator;
    public List<SwerveModule> m_swerveModules;
    public Pigeon2 m_imu;
    public Field2d m_fieldDisplay;

    public Swerve() {
        m_imu = new Pigeon2(Constants.Swerve.pigeonID);
        m_imu.configFactoryDefault();
        zeroGyro();

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

        // TODO tune the filter
        m_swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
    }

    public void drive(
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

        for (final var mod : m_swerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
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
        for (final var mod : m_swerveModules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (final var mod : m_swerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        m_imu.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_imu.getYaw()) : Rotation2d.fromDegrees(m_imu.getYaw());
    }

    public void resetModulesToAbsolute(){
        for (final var mod : m_swerveModules) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        m_swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        LimelightHelpers.getBotpose_wpiAlliance("").map(Pose3d::toPose2d).ifPresent(measurement -> {
            final var imageCaptureTime = LimelightHelpers.getLatency_Pipeline("");
            m_swervePoseEstimator.addVisionMeasurement(measurement, Timer.getFPGATimestamp() - imageCaptureTime);
        });

        m_fieldDisplay.setRobotPose(m_swervePoseEstimator.getEstimatedPosition());

        for (final var mod : m_swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}