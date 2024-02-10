// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.crypto.spec.RC2ParameterSpec;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDGains;
import frc.robot.Constants.IntakeProfile;

public class Intake extends SubsystemBase {
  private CANSparkMax m_wrist = new CANSparkMax(IntakeProfile.wristID, MotorType.kBrushless);
  private WPI_TalonSRX m_overRoller = new WPI_TalonSRX(IntakeProfile.overRoller);
  private WPI_TalonSRX m_underRoller = new WPI_TalonSRX(IntakeProfile.underRoller);

  private RelativeEncoder wristEncoder = m_wrist.getEncoder(SparkRelativeEncoder.Type.kHallSensor, IntakeProfile.neoEncoderCountsPerRev);

  private SparkPIDController wristController = m_wrist.getPIDController();

  private TimeOfFlight s_DistanceSensor = new TimeOfFlight(0);

  /** Creates a new Intake. */
  public Intake() {
    m_wrist.restoreFactoryDefaults();
    m_overRoller.configFactoryDefault();
    m_underRoller.configFactoryDefault();

    m_wrist.clearFaults();

    m_wrist.setInverted(false); //TODO: Ensure intake moves outward
    m_overRoller.setInverted(false); //TODO: Ensure roller spins inward
    m_underRoller.setInverted(false); //TODO: Ensure roller spins inward

    m_wrist.setSmartCurrentLimit(IntakeProfile.kWristCurrentLimit);
    m_overRoller.configPeakCurrentLimit(IntakeProfile.kRollerCurrentLimit);

    m_wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_wrist.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_wrist.setSoftLimit(SoftLimitDirection.kReverse, (float)IntakeProfile.kInitialPos);
    m_wrist.setSoftLimit(SoftLimitDirection.kForward, (float)IntakeProfile.kDeployedPos);

    m_wrist.setIdleMode(IdleMode.kBrake);

    wristEncoder.setPositionConversionFactor(IntakeProfile.kPositionFactor);
    wristEncoder.setVelocityConversionFactor(IntakeProfile.kVelocityFactor);
    wristEncoder.setPosition(IntakeProfile.kInitialPos);

    PIDGains.setSparkMaxGains(wristController, IntakeProfile.kWristPositionGains);

    m_wrist.burnFlash();
  }

  private void deployIntake() {
    double encoderCounts = wristEncoder.getCountsPerRevolution();
    double deployedPos = 1000;
    if (encoderCounts >= deployedPos) {
      m_wrist.set(0);
      m_wrist.setIdleMode(IdleMode.kCoast);
    }
    else {
      m_wrist.set(0.2);
    }
  }

  private void undeployIntake() {
    double encoderCounts = wristEncoder.getCountsPerRevolution();
    double deployedPos = 0;
    if (encoderCounts <= deployedPos) {
      m_wrist.set(0);
      m_wrist.setIdleMode(IdleMode.kBrake);
    }
    else {
      m_wrist.set(0.2);
    }
  }

  public void enableIntakePlus(boolean enable) {
    if (!enable) {
      undeployIntake();
    }
    else {
      deployIntake();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
