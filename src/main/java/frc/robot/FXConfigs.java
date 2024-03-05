package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ClimberProfile;
import frc.robot.Constants.SwerveProfile;

public final class FXConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public TalonFXConfiguration climberFXConfigA = new TalonFXConfiguration();
    public TalonFXConfiguration climberFXConfigB = new TalonFXConfiguration();

    public FXConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SwerveProfile.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = SwerveProfile.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = SwerveProfile.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveProfile.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveProfile.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveProfile.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveProfile.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveProfile.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = SwerveProfile.angleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveProfile.angleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveProfile.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = SwerveProfile.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveProfile.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveProfile.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveProfile.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveProfile.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveProfile.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveProfile.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = SwerveProfile.driveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveProfile.driveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveProfile.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveProfile.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveProfile.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveProfile.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveProfile.closedLoopRamp;

        /** Climber Motor A Configuration */
        /* Motor Inverts and Neutral Mode */
        climberFXConfigA.MotorOutput.Inverted = ClimberProfile.climberAMotorInvert;
        climberFXConfigA.MotorOutput.NeutralMode = ClimberProfile.climberNeutralMode;

        /* Current Limiting */
        climberFXConfigA.CurrentLimits.SupplyCurrentLimitEnable = ClimberProfile.climberEnableCurrentLimit;
        climberFXConfigA.CurrentLimits.SupplyCurrentLimit = ClimberProfile.climberCurrentLimit;
        climberFXConfigA.CurrentLimits.SupplyCurrentThreshold = ClimberProfile.climberCurrentThreshold;
        climberFXConfigA.CurrentLimits.SupplyTimeThreshold = ClimberProfile.climberCurrentThresholdTime;

        /* PID Config */
        climberFXConfigA.Slot0.kP = ClimberProfile.climberKP;
        climberFXConfigA.Slot0.kI = ClimberProfile.climberKI;
        climberFXConfigA.Slot0.kD = ClimberProfile.climberKD;
        climberFXConfigA.Slot0.kD = ClimberProfile.climberKF;

        /* Soft Limits */
        climberFXConfigA.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberProfile.climberEnableRVSSoftLimit;
        climberFXConfigA.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberProfile.climberRVSSoftLimitThreshold;
        climberFXConfigA.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberProfile.climberEnableFWDSoftLimit;
        climberFXConfigA.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberProfile.climberFWDSoftLimitThreshold;

        /** Climber Motor B Configuration */
        /* Motor Inverts and Neutral Mode */
        climberFXConfigB.MotorOutput.Inverted = ClimberProfile.climberBMotorInvert;
        climberFXConfigB.MotorOutput.NeutralMode = ClimberProfile.climberNeutralMode;

        /* Current Limiting */
        climberFXConfigB.CurrentLimits.SupplyCurrentLimitEnable = ClimberProfile.climberEnableCurrentLimit;
        climberFXConfigB.CurrentLimits.SupplyCurrentLimit = ClimberProfile.climberCurrentLimit;
        climberFXConfigB.CurrentLimits.SupplyCurrentThreshold = ClimberProfile.climberCurrentThreshold;
        climberFXConfigB.CurrentLimits.SupplyTimeThreshold = ClimberProfile.climberCurrentThresholdTime;

        /* PID Config */
        climberFXConfigB.Slot0.kP = ClimberProfile.climberKP;
        climberFXConfigB.Slot0.kI = ClimberProfile.climberKI;
        climberFXConfigB.Slot0.kD = ClimberProfile.climberKD;
        climberFXConfigB.Slot0.kD = ClimberProfile.climberKF;

        /* Soft Limits */
        climberFXConfigB.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimberProfile.climberEnableRVSSoftLimit;
        climberFXConfigB.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberProfile.climberRVSSoftLimitThreshold;
        climberFXConfigB.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimberProfile.climberEnableFWDSoftLimit;
        climberFXConfigB.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberProfile.climberFWDSoftLimitThreshold;
    }
}