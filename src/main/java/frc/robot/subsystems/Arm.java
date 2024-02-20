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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.LinearInterpolation;
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

  private DigitalInput indexorSensor = new DigitalInput(ArmProfile.noteDetectorChannel);
  private boolean noteDetected = !indexorSensor.get();

  /** Creates a new Arm. */
  public Arm() {
    m_pivotA.restoreFactoryDefaults();
    m_pivotB.restoreFactoryDefaults();
    m_indexor.configFactoryDefault();
    m_shooterA.configFactoryDefault();
    m_shooterB.configFactoryDefault();

    m_pivotA.setInverted(false);
    m_pivotB.setInverted(true);
    m_indexor.setInverted(false);//TODO: Ensure belts run up
    m_shooterA.setInverted(false);
    m_shooterB.setInverted(true);//TODO: Ensure wheels spin outward

    m_pivotA.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);
    m_pivotB.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);

    m_pivotA.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotA.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotA.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitRvs);
    m_pivotB.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitRvs);
    m_pivotA.setSoftLimit(SoftLimitDirection.kForward, (float)ArmProfile.kPivotSoftLiimitFwd); //TODO: Ensure foward limit is at 90 degrees
    m_pivotB.setSoftLimit(SoftLimitDirection.kForward, (float)ArmProfile.kPivotSoftLiimitFwd);

    m_pivotA.setIdleMode(IdleMode.kBrake);
    m_pivotB.setIdleMode(IdleMode.kBrake);

    pivotEncoderA.setPositionConversionFactor(ArmProfile.kPositionFactor);
    pivotEncoderB.setVelocityConversionFactor(ArmProfile.kVelocityFactor);
    pivotEncoderA.setPosition(ArmProfile.pivotInitialPos);
    pivotEncoderB.setPosition(ArmProfile.pivotInitialPos);

    PIDGains.setSparkMaxGains(pivotControllerA, ArmProfile.kArmPositionGains);
    PIDGains.setSparkMaxGains(pivotControllerB, ArmProfile.kArmPositionGains);

    m_pivotA.burnFlash();
    m_pivotB.burnFlash();

    setArmPos(ArmProfile.pivotInitialPos);
  }

  public boolean isNoteDetected() {
    return noteDetected;
  }

  private double pivotEncoderCountsToDegrees(double encoderInput) {
    double posIndegrees = (ArmProfile.kArmGearRatio * encoderInput) / 360;
    return posIndegrees;
  }

  public void setArmPos(double commandedOutputDegree) {
    double lowerLimit = commandedOutputDegree - 0.1;
    double upperLimit = commandedOutputDegree + 0.1;
    double currentAngle = pivotEncoderCountsToDegrees(pivotEncoderA.getCountsPerRevolution());
    if ((lowerLimit <= currentAngle) && (currentAngle <= upperLimit)) {
      setArmOutput(0);
    }
    else if (currentAngle < lowerLimit) {
      setArmOutput(ArmProfile.kArmDefaultOutput);
    }
    else {
      setArmOutput(ArmProfile.kArmDefaultOutput * -0.5); // 40% negitive output
    }
  }

  private double getPivotDegreeForDistance(double targetdistance){
    // setPivotAngle(distance);
    double requiredDegree = LinearInterpolation.linearInterpolation(ArmProfile.TargetDistanceArray, ArmProfile.ArmDegreeArray, targetdistance);
    return requiredDegree;
  }

  public void prepareToShoot(Swerve s_Swerve, Vision s_Vision, Intake s_Intake) {
    setShooterOutput(1);
    if (s_Vision.LinedUpWithSpeaker()) {
      s_Intake.explodeForShooter();
    }
  }

  public void fireAtTarget(Vision s_Vision) {
    double commandedOutputDegree = getPivotDegreeForDistance(s_Vision.getFilteredDistance());
    double lowerLimit = commandedOutputDegree - 0.1;
    double upperLimit = commandedOutputDegree + 0.1;
    double currentAngle = pivotEncoderCountsToDegrees(pivotEncoderA.getCountsPerRevolution());
    if ((lowerLimit <= currentAngle) && (currentAngle <= upperLimit)) {
      setArmOutput(0);
      setIndexorOuput(1);
    }
    else if (currentAngle < lowerLimit) {
      setArmOutput(ArmProfile.kArmDefaultOutput);
    }
    else {
      setArmOutput(ArmProfile.kArmDefaultOutput * -0.4); // 32% negitive output
    }
  }

  public void setArmOutput(double commandedOutputFraction) {
    m_pivotA.set(commandedOutputFraction);
    m_pivotB.set(commandedOutputFraction);
  }

  public void setIndexorOuput(double commandedOutputFraction) {
    m_indexor.set(commandedOutputFraction);
  }

  public void setShooterOutput(double commandedOutputFraction) {
    m_shooterA.set(commandedOutputFraction);
    m_shooterB.set(commandedOutputFraction);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note Collected", isNoteDetected());
    
    SmartDashboard.putNumber("Neo Current (A) ", m_pivotA.getOutputCurrent());
    SmartDashboard.putNumber("Neo Current (B)", m_pivotB.getOutputCurrent());
    SmartDashboard.putNumber("Combined Neo Current", m_pivotA.getOutputCurrent() + m_pivotB.getOutputCurrent());
  }
}
