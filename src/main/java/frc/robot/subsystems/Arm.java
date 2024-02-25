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
  private boolean armIsReset = false;

  private SparkPIDController pivotControllerA = m_pivotA.getPIDController();
  private SparkPIDController pivotControllerB = m_pivotB.getPIDController();

  private DigitalInput indexorSensor = new DigitalInput(ArmProfile.noteDetectorChannel);
  private boolean noteDetected = false;

  private boolean armIsClearForClimb = false;

  /** Creates a new Arm. */
  public Arm() {
    m_pivotA.restoreFactoryDefaults();
    m_pivotB.restoreFactoryDefaults();
    m_indexor.configFactoryDefault();
    m_shooterA.configFactoryDefault();
    m_shooterB.configFactoryDefault();
 
    m_pivotA.setInverted(true);
    m_pivotB.setInverted(false);
    m_indexor.setInverted(true);
    m_shooterA.setInverted(false);
    m_shooterB.setInverted(false);

    m_pivotA.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);
    m_pivotB.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);

    m_pivotA.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotA.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotA.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitRvs);
    m_pivotB.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitRvs);
    m_pivotA.setSoftLimit(SoftLimitDirection.kForward, (float)ArmProfile.kPivotSoftLiimitFwd);
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
  }

  public boolean isNoteDetected() {
    return noteDetected;
  }

  public void setArmPos(double commandedOutputDegree) {
    double lowerLimit = commandedOutputDegree - ArmProfile.kPivotDegreeThreshold;
    double upperLimit = commandedOutputDegree + ArmProfile.kPivotDegreeThreshold;
    double currentPos = pivotEncoderA.getPosition();
    if ((lowerLimit <= currentPos) && (currentPos <= upperLimit)) {
      setArmOutput(0);
    }
    else if (currentPos <= lowerLimit) {
      setArmOutput(ArmProfile.kArmDefaultOutput);
    }
    else {
      setArmOutput(ArmProfile.kArmDefaultOutput * -0.25); // 20% negitive output
    }
  }

  public void prepareToShoot(Swerve s_Swerve, Vision s_Vision, Intake s_Intake) {
    setShooterOutput(ArmProfile.kShooterDefaultOutput);
    //if (s_Vision.LinedUpWithSpeaker()) {
      s_Intake.deployPlus();
    //}
  }

  private double getPivotDegreeForDistance(double targetdistance){
    double requiredDegree = LinearInterpolation.linearInterpolation(ArmProfile.TargetDistanceArray, ArmProfile.ArmDegreeArray, targetdistance);
    return requiredDegree;
  }

  public void fireAtTarget(Vision s_Vision) {
    //double commandedOutputDegree = getPivotDegreeForDistance(s_Vision.getFilteredDistance());
    double commandedOutputPos = 4000; //TODO: Must be configured for testing/showcasing
    double lowerLimit = commandedOutputPos - ArmProfile.kPivotDegreeThreshold;
    double upperLimit = commandedOutputPos + ArmProfile.kPivotDegreeThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      setArmOutput(0);
      setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
    else if (pivotEncoderA.getPosition() <= lowerLimit) {
      setArmOutput(ArmProfile.kArmDefaultOutput);
    }
    else {
      setArmOutput(ArmProfile.kArmDefaultOutput * -0.25); // 20% negitive output
    }
  }

  public void resetArm() {
    setArmPos(ArmProfile.pivotInitialPos);
    setIndexorOuput(0);
    setShooterOutput(0);
    armIsReset = true;
  }

  public boolean isArmReset() {
    if (pivotEncoderA.getPosition() <= ArmProfile.pivotInitialPos + ArmProfile.kPivotDegreeThreshold) {
      return true;
    }
    else {
      return false;
    }
  }

  public void setArmToClimbPos() {
    setArmPos(ArmProfile.kPivotClimbPos);
    if (pivotEncoderA.getPosition() >= ArmProfile.kPivotClimbPos) {
      armIsClearForClimb = true;
    }
  }

  public boolean isArmClearForClimb() {
    return armIsClearForClimb;
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
    
    SmartDashboard.putNumber("Arm Degree", pivotEncoderA.getPosition());
    SmartDashboard.putNumber("Neo Current (A) ", m_pivotA.getOutputCurrent());
    SmartDashboard.putNumber("Neo Current (B)", m_pivotB.getOutputCurrent());
    SmartDashboard.putNumber("Combined Neo Current", m_pivotA.getOutputCurrent() + m_pivotB.getOutputCurrent());
  }
}
