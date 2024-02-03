// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDGains;
import frc.robot.Constants.ArmProfile;

public class Arm extends SubsystemBase {
  private CANSparkMax m_pivotA = new CANSparkMax(ArmProfile.pivotMotorID_A, MotorType.kBrushless);
  private CANSparkMax m_pivotB = new CANSparkMax(ArmProfile.pivotMotorID_B, MotorType.kBrushless);
  private WPI_TalonSRX m_indexor = new WPI_TalonSRX(ArmProfile.indexorID);
  private WPI_TalonSRX m_shooterA = new WPI_TalonSRX(ArmProfile.shooterID_A);
  private WPI_TalonSRX m_shooterB = new WPI_TalonSRX(ArmProfile.shooterID_B);

  private RelativeEncoder pivotEncoderA = m_pivotA.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);
  private RelativeEncoder pivotEncoderB = m_pivotB.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);

  private SparkPIDController pivotControllerA = m_pivotA.getPIDController();
  private SparkPIDController pivotControllerB = m_pivotB.getPIDController();

  /** Creates a new Arm. */
  public Arm() {
    m_pivotA.restoreFactoryDefaults();
    m_pivotB.restoreFactoryDefaults();
    m_indexor.configFactoryDefault();
    m_shooterA.configFactoryDefault();
    m_shooterB.configFactoryDefault();

    m_pivotA.setInverted(false);
    m_pivotB.setInverted(true); //TODO: Ensure arm goes up
    m_indexor.setInverted(false);//TODO: Ensure belts run up
    m_shooterA.setInverted(false);
    m_shooterB.setInverted(true);//TODO: Ensure wheels spin outward

    m_pivotA.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);
    m_pivotB.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);

    m_pivotA.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotA.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_pivotA.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitReverse);
    m_pivotB.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitReverse);

    m_pivotA.setIdleMode(IdleMode.kBrake);
    m_pivotB.setIdleMode(IdleMode.kBrake);

    pivotEncoderA.setPositionConversionFactor(ArmProfile.kPositionFactor);
    pivotEncoderB.setVelocityConversionFactor(ArmProfile.kVelocityFactor);
    pivotEncoderA.setPosition(ArmProfile.kPivotInitialPos);
    pivotEncoderB.setPosition(ArmProfile.kPivotInitialPos);

    PIDGains.setSparkMaxGains(pivotControllerA, ArmProfile.kArmPositionGains);
    PIDGains.setSparkMaxGains(pivotControllerB, ArmProfile.kArmPositionGains);

    m_pivotA.burnFlash();
    m_pivotB.burnFlash();
  }

  public void runArmOutput(double commandedOutputFraction) {
    m_pivotA.set(commandedOutputFraction);
    m_pivotB.set(commandedOutputFraction);
  }

  public void runIndexorOuput(double commandedOutputFraction) {
    m_indexor.set(commandedOutputFraction);
  }

  public void runShooterOutput(double commandedOutputFraction) {
    m_shooterA.set(commandedOutputFraction);
    m_shooterB.set(commandedOutputFraction);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
