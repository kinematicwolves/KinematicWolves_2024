// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.PIDGains;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static class ControllerProfile {
    /* Controller Port Assingments */
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final int kTechnitionControllerPort = 2;

    /* Joystick Deadband */
    public static final double stickDeadband = 0.15;
  }

  public static class ArmProfile {
    /* Id's and Port Assingments */
    public static final int pivotMotorID_A = 15;
    public static final int pivotMotorID_B = 16;
    public static final int indexorID = 17;
    public static final int shooterID_A = 18;
    public static final int shooterID_B = 19;
    public static final int irSensorPort = 9; //TODO: Must be configured

    /* Current Limiting */
    public static final int kPivotCurrentLimit = 20;

    /* Soft Limits */
    public static final double kPivotSoftLimitRvs = 0.0;
    public static final double kPivotSoftLiimitFwd = 54000;

    /* Arm Threshold and Setpoints */
    public static final double kPivotPosThreshold = 550;
    public static final double pivotInitialPos = 0;
    public static final double kpivotSpeakerPos = 13000;
    public static final double kpivotAmpPos = 53500; //54700;

    /* Conversion Factors */
    public static final int neoEncoderCountsPerRev = 42;
    public static final double kArmGearRatio = ((3.0 * 4.0 * 5.0) * 3.0) / 1;
    public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;

    /* Set Outputs */
    public static final double kIndexorDefaultOutput = 100;
    public static final double kShooterDefaultOutput = 90;
    public static final double kShooterAmpOutput = 40;

    /* Arm PID Gains */
    public static final PIDGains kArmPositionGains = new PIDGains(0.1, 0.0, 0.0);
  }

  public static class IntakeProfile {
    /* Id's */
    public static final int wristID = 20;
    public static final int outerRoller = 21;
    public static final int innerRoller = 22;

    /* Current Limiting */
    public static final int kWristCurrentLimit = 25;
    public static final int kOuterRollerCurrentLimit = 25;
    public static final int kInnerRollerCurrentLimit = 25;

    /* Wrist Threshold and Setpoints */ //TODO: Must be configured for new through bore sensor
    public static final double wristPosOffset = -0.121;
    public static final double wristPosInversion = 1; // Or -1. Down Positive
    public static final double kWristThreshhold = 0.02;
    public static final double kInitialPos = 0.001 + kWristThreshhold;
    public static final double kDeployedPos = 0.24 - kWristThreshhold;
    public static final double noteDetectedDistance = 110; // Distance in millimeters

    /* Conversion Factors */
    public static final int neoEncoderCountsPerRev = 42;    
    public static final double kWristGearRatio = 70 / 1;
    public static final double kPositionFactor = kWristGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kWristGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 11000 * kVelocityFactor;

    /* Wrist PID Gains */
    public static final PIDGains kWristPositionGains = new PIDGains(0.1, 0.0, 0.0);

    /* Default Percent Outputs */
    public static final double kOuterDefaultOutput = 90;
    public static final double kInnerDefaultOutput = 100;
    public static final double kWristDefaultOutput = 21;
    public static final double kWristSlowOutput = 11;
  }

  public static class ClimberProfile {
    /* Id's */
    public static final int climberA_ID = 30; 
    public static final int climberB_ID = 31;

    /* Motor Inverts */ 
    public static final InvertedValue climberAMotorInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue climberBMotorInvert = InvertedValue.Clockwise_Positive;

    /* Neutral Modes */ 
    public static final NeutralModeValue climberNeutralMode = NeutralModeValue.Coast;

    /* Climber Current Limiting */
    public static final int climberCurrentLimit = 40;
    public static final int climberCurrentThreshold = 40;
    public static final double climberCurrentThresholdTime = 0.1;
    public static final boolean climberEnableCurrentLimit = true;

    /* Soft Limits */
    public static final boolean climberEnableRVSSoftLimit = true;
    public static final double climberRVSSoftLimitThreshold = 0;
    public static final boolean climberEnableFWDSoftLimit = false;
    public static final double climberFWDSoftLimitThreshold = 335;
   
    /* Climber Motor PID Values */ //TODO: These must be configured
    public static final double climberKP = 0.0;
    public static final double climberKI = 0.0;
    public static final double climberKD = 0.0;
    public static final double climberKF = 0.0;

    /* Threshold and Setpoints */
    public static final double climberMaxHeightPos = 182;
    public static final double climberTrueMaxPos = 333;

    /* Climber Set Outputs */
    public static final double climberDefaultOutput = 0.2;
    public static final double outputWithZeroLoad = 0.4;
    public static final double outputWithRobotLoad = 0.75;
  }

  public static class LightingProfile {
    /* Id's */
    public static final int candldeID = 23;
    
    /* Brightness */
    public static final double kBrightnessScalar = 1;

    /* Total Number of Led's */
    public static final int numLEDStrip = 88;
  }

  public static final class SwerveProfile {
        public static final int pigeonID = 13;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.5); // Center to Center distance of left and right modules wheels in meters.
        public static final double wheelBase = Units.inchesToMeters(19.5); // Center to Center distance of front and rear module wheels in meters.
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 30;
        public static final int driveCurrentThreshold = 50;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 2.0;
        public static final double closedLoopRamp = 0.1;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.1;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0; //TODO: This must be configured
        public static final double driveKV = 0;
        public static final double driveKA = 0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 0.0023;// // 4.99m/s before weight
        /** Radians per Second */
        public static final double maxAngularVelocity = 0.0027; // 13.99r/s before weight
        /** Fractional Percentage **/
        public static final double speedCap = 0.35; // Must be configured

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
        /* Module Specific Constants */
        /* Back Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(135.17);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Left Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-82.96);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-95.36);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(5.87);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
// print("kinematic wolves rulez")
        /* Backup Moudles Specific Constants */
        /** Backup Module 1: TODO: This must be configured
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
        ** Mod 0 Offset:
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        ** Mod 1 Offset:
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        ** Mod 2 Offset:
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        ** Mod 3 Offset:
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        
        /** Backup Module 2: TODO: This must be configured
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
        ** Mod 0 Offset:
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        ** Mod 1 Offset:
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        ** Mod 2 Offset:
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        ** Mod 3 Offset:
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        */
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
