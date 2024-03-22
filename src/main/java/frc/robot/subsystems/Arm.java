// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDGains;
import frc.robot.Constants.ArmProfile;
import frc.robot.Constants.IntakeProfile;

public class Arm extends SubsystemBase {
  private CANSparkMax m_pivotA = new CANSparkMax(ArmProfile.pivotMotorID_A, MotorType.kBrushless);
  private CANSparkMax m_pivotB = new CANSparkMax(ArmProfile.pivotMotorID_B, MotorType.kBrushless);
  private WPI_TalonSRX m_indexor = new WPI_TalonSRX(ArmProfile.indexorID);
  private WPI_TalonSRX m_shooterA = new WPI_TalonSRX(ArmProfile.shooterID_A);
  private WPI_TalonSRX m_shooterB = new WPI_TalonSRX(ArmProfile.shooterID_B);

  //private WPI_VictorSPX
  private RelativeEncoder pivotEncoderA = m_pivotA.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);
  private RelativeEncoder pivotEncoderB = m_pivotB.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);

  private SparkPIDController pivotControllerA = m_pivotA.getPIDController();
  private SparkPIDController pivotControllerB = m_pivotB.getPIDController();

  private TimeOfFlight indexorSensor = new TimeOfFlight(0);

  /** Creates a new Arm. */
  public Arm() {
    /* Initaliaztion Box */
    /* Factory Resets */
    m_pivotA.restoreFactoryDefaults();
    m_pivotB.restoreFactoryDefaults();
    m_indexor.configFactoryDefault();
    m_shooterA.configFactoryDefault();
    m_shooterB.configFactoryDefault();
 
    /* Inversion Factors */
    m_pivotA.setInverted(true);
    m_pivotB.setInverted(false);
    m_indexor.setInverted(true);
    m_shooterA.setInverted(false);
    m_shooterB.setInverted(false);

    /* Arm Current Limiting */
    m_pivotA.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);
    m_pivotB.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);

    /* Arm Software Limiting */
    m_pivotA.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotA.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotA.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitRvs);
    m_pivotB.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitRvs);
    m_pivotA.setSoftLimit(SoftLimitDirection.kForward, (float)ArmProfile.kPivotSoftLiimitFwd);
    m_pivotB.setSoftLimit(SoftLimitDirection.kForward, (float)ArmProfile.kPivotSoftLiimitFwd);

    /* Arm Neutral Modes */
    m_pivotA.setIdleMode(IdleMode.kBrake);
    m_pivotB.setIdleMode(IdleMode.kBrake);

    /* Conversion Factors */
    pivotEncoderA.setPositionConversionFactor(ArmProfile.kPositionFactor);
    pivotEncoderB.setVelocityConversionFactor(ArmProfile.kVelocityFactor);
    pivotEncoderA.setPosition(ArmProfile.pivotInitialPos);
    pivotEncoderB.setPosition(ArmProfile.pivotInitialPos);

    /* Arm PID Gains */
    PIDGains.setSparkMaxGains(pivotControllerA, ArmProfile.kArmPositionGains);
    PIDGains.setSparkMaxGains(pivotControllerB, ArmProfile.kArmPositionGains);

    /* Flash Arm Controllers with Set Config */
    m_pivotA.burnFlash();
    m_pivotB.burnFlash();
  }

  public void setArmPivotPos(double commandedPivotPos, double upwardOutput, double downwardOutput) {
    double lowerLimit = commandedPivotPos - ArmProfile.kPivotPosThreshold;
    double upperLimit = commandedPivotPos + ArmProfile.kPivotPosThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      setArmOutput(0);
    }
    else if (pivotEncoderA.getPosition() <= lowerLimit) {
      setArmOutput(upwardOutput);
    }
    else {
      setArmOutput(-downwardOutput);
    }
  }

  public void prepareToShoot(Intake s_Intake) {
    setShooterOutput(ArmProfile.kShooterDefaultOutput);
    s_Intake.deployPlus();
  }

  public void prepareToDump(Intake s_Intake) {
    s_Intake.deployPlus();
    m_shooterA.set(ArmProfile.kShooterAmpOutput * 0.2);
    m_shooterB.set(ArmProfile.kShooterAmpOutput);
  }

  public void fireAtSetPos(double commandedPos, double upwardOutput, double downwardOutput) {
    double lowerLimit = commandedPos - ArmProfile.kPivotPosThreshold;
    double upperLimit = commandedPos + ArmProfile.kPivotPosThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      setArmOutput(0);
      setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
    else if (pivotEncoderA.getPosition() <= lowerLimit) {
      setArmOutput(upwardOutput);
    }
    else {
      setArmOutput(-downwardOutput);
    }
  }

  public void resetArmPivot() {
    setIndexorOuput(0);
    setShooterOutput(0);
    double upperLimit = ArmProfile.pivotInitialPos + ArmProfile.kPivotPosThreshold;
    double currentPos = pivotEncoderA.getPosition();
    if (currentPos <= upperLimit) {
      setArmOutput(0);
    }
    else {
      setArmOutput(-0.4);
    }
  }

  public boolean isNoteStowed() {
    if (indexorSensor.getRange() <= 80) {
      return true;
    }
    else {
      return false;
    }
  }

  public void stowNote(Intake s_Intake) {
    if (isNoteStowed() == true) {
      m_indexor.set(0);
      s_Intake.setInnerRollerOutput(0);
    }
    else {
      m_indexor.set(ArmProfile.kIndexorDefaultOutput);
      s_Intake.setInnerRollerOutput(IntakeProfile.kInnerDefaultOutput);
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
    SmartDashboard.putBoolean("isNoteStored", isNoteStowed());
    SmartDashboard.putNumber("Arm Position", pivotEncoderA.getPosition());
    SmartDashboard.putNumber("Indexor Current", m_indexor.getSupplyCurrent());
    SmartDashboard.putNumber("Arm Motor Currents ", m_pivotA.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Motor Currents", m_shooterA.getSupplyCurrent());
  }
}
