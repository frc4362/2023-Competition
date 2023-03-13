package com.gemsrobotics.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import com.gemsrobotics.lib.util.COTSFalconSwerveConstants;
import com.gemsrobotics.lib.util.SwerveModuleConstants;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;


public final class Constants {

    public static final double stickDeadband = 0.15;
    public static final double VISION_OUTLIER_DISTANCE = 1.0; // meters
	public static final double DEBOUNCE_TIME_SECONDS = 0.1;
    public static final int PILOT_PORT = 0;
    public static final int COPILOT_PORT = 1;

    public static final class CANBusses {
        public static final String AUX = "aux";
        public static final String MAIN = "rio";
    }

    public static final class Features {
        public static final boolean DO_VISION_FILTER = false;
    }

    public static final CTREConfigs CTRE_CONFIGS = new CTREConfigs();

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  COTSFalconSwerveConstants.SwerveX();

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.5); //: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(22.5); //: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.056; //: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.48 / 12); //: This must be tuned to specific robot
        public static final double driveKV = (1.87 / 12);
        public static final double driveKA = (0.26 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.95; //: This must be tuned to specific robot

        /** Analytically derived, Radians per Second */
        public static final double maxAngularVelocity = (maxSpeed / Math.hypot(trackWidth / 2.0, wheelBase / 2.0)) / 2.0;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Back Left Module - Module 2 */ // Used to be 0
        public static final class Mod2 { //: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(277.25);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, CANBusses.AUX);
        }

        /* Back Right Module - Module 3 */ //Used to be 1
        public static final class Mod3 { //: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(234.18);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, CANBusses.AUX);
        }
        
        /* Front Left Module - Module 0 */ //Used to be 2
        public static final class Mod0 { //: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(288.00);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, CANBusses.AUX);
        }

        /* Front Right Module - Module 1 */ // Used to be 3
        public static final class Mod1 { //: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(23.15);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, CANBusses.AUX);
        }
    }

    public static final class AutoConstants { //: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.95;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 5;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Generation {
        public static final PathConstraints constraints =
                new PathConstraints(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

        public static final ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController,
                0,
                0,
                Constants.AutoConstants.kThetaControllerConstraints);

        public static final PIDController pureThetaController = new PIDController(
                AutoConstants.kPThetaController,
                0,
                0
        );

        public static final double maxCentripetalAcceleration = 3.0;

        static {
//            configForward.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAcceleration));
//            configBackward.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAcceleration));
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
            pureThetaController.enableContinuousInput(-Math.PI, Math.PI);
        }

//        public static Trajectory getForwardTrajectory(
//                final Pose2d starterPose,
//                final List<Translation2d> poses,
//                final Pose2d endPose
//        ) {
//            return TrajectoryGenerator.generateTrajectory(
//                    starterPose,
//                    poses,
//                    endPose,
//                    configForward
//            );
//        }

//        public static Trajectory getForwardTrajectory(final Pose2d starterPose, final Pose2d endPose) {
//            return getForwardTrajectory(starterPose, List.of(), endPose);
//        }
//
//        public static Trajectory getBackwardTrajectory(
//                final Pose2d starterPose,
//                final List<Translation2d> poses,
//                final Pose2d endPose
//        ) {
//            return TrajectoryGenerator.generateTrajectory(
//                    starterPose,
//                    poses,
//                    endPose,
//                    configBackward
//            );
//        }
//
//        public static Trajectory getBackwardTrajectory(final Pose2d starterPose, final Pose2d endPose) {
//            return getBackwardTrajectory(starterPose, List.of(), endPose);
//        }
    }
}
