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
import frc.lib.util.PIDGains;
import frc.robot.Constants.ArmProfile;
import frc.robot.Constants.IntakeProfile;

public class Arm extends SubsystemBase {
  /** Pivot Motors */
  private CANSparkMax m_pivotA = new CANSparkMax(ArmProfile.pivotMotorID_A, MotorType.kBrushless);
  private CANSparkMax m_pivotB = new CANSparkMax(ArmProfile.pivotMotorID_B, MotorType.kBrushless);

  /** Indexor Motor */
  private WPI_TalonSRX m_indexor = new WPI_TalonSRX(ArmProfile.indexorID);

  /** Shooter Motors */
  private WPI_TalonSRX m_shooterA = new WPI_TalonSRX(ArmProfile.shooterID_A);
  private WPI_TalonSRX m_shooterB = new WPI_TalonSRX(ArmProfile.shooterID_B);

  /** Pivot Motor Controllers */
  private RelativeEncoder pivotEncoderA = m_pivotA.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);
  private RelativeEncoder pivotEncoderB = m_pivotB.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);

  /** Pivot PID Controllers */
  private SparkPIDController pivotControllerA = m_pivotA.getPIDController();
  private SparkPIDController pivotControllerB = m_pivotB.getPIDController();

  /** Indexor Sensor */
  private DigitalInput beamBreakSensor = new DigitalInput(ArmProfile.irSensorPort);

  /** Initaliaztion Box */
  public Arm() {
    /** Factory Resets */
    m_pivotA.restoreFactoryDefaults();
    m_pivotB.restoreFactoryDefaults();
    m_indexor.configFactoryDefault();
    m_shooterA.configFactoryDefault();
    m_shooterB.configFactoryDefault();
 
    /** Inversion Factors */
    m_pivotA.setInverted(true);
    m_pivotB.setInverted(false);
    m_indexor.setInverted(true);
    m_shooterA.setInverted(false);
    m_shooterB.setInverted(false);

    /** Arm Current Limits */
    m_pivotA.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);
    m_pivotB.setSmartCurrentLimit(ArmProfile.kPivotCurrentLimit);

    /** Arm Software Limits */
    m_pivotA.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotA.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_pivotA.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitRvs);
    m_pivotB.setSoftLimit(SoftLimitDirection.kReverse, (float)ArmProfile.kPivotSoftLimitRvs);
    m_pivotA.setSoftLimit(SoftLimitDirection.kForward, (float)ArmProfile.kPivotSoftLiimitFwd);
    m_pivotB.setSoftLimit(SoftLimitDirection.kForward, (float)ArmProfile.kPivotSoftLiimitFwd);

    /** Arm Neutral Modes */
    m_pivotA.setIdleMode(IdleMode.kBrake);
    m_pivotB.setIdleMode(IdleMode.kBrake);

    /** Conversion Factors */
    pivotEncoderA.setPositionConversionFactor(ArmProfile.kPositionFactor);
    pivotEncoderB.setVelocityConversionFactor(ArmProfile.kVelocityFactor);
    pivotEncoderA.setPosition(ArmProfile.pivotInitialPos);
    pivotEncoderB.setPosition(ArmProfile.pivotInitialPos);

    /** Arm PID Gains */
    PIDGains.setSparkMaxGains(pivotControllerA, ArmProfile.kArmPositionGains);
    PIDGains.setSparkMaxGains(pivotControllerB, ArmProfile.kArmPositionGains);

    /** Flash Arm Controllers Configs */
    m_pivotA.burnFlash();
    m_pivotB.burnFlash();
  }

  /**
   * Sets shooter speed at 90%
   * 
   * @param s_Intake Calls intake subsystem to deploy++
   */
  public void prepareToShoot(Intake s_Intake) {
    setShooterOutput(ArmProfile.kShooterDefaultOutput);
    s_Intake.deployPlus();
  }

  /**
   * Sets upper flywheels to 40% 
   * Sets lower flywheeks to 8%
   * 
   * @param s_Intake Calls intake subsystem to deploy intake++
   */
  public void prepareToDump(Intake s_Intake) {
    double percentageSpeedDecay = 20;

    m_shooterA.set(ArmProfile.kShooterAmpOutput * percentageSpeedDecay);
    m_shooterB.set(ArmProfile.kShooterAmpOutput);
    s_Intake.deployPlus();
  }

  /**
   * Checks if arm is at commanded encoder count
   * 
   * @param commandedPivotPos pivot encoder input
   * @return true if arm pivot is in between position thresholds, otherwise false
   */
  public boolean armAtSetPosition(double commandedPivotPos) {
    double lowerLimit = commandedPivotPos - ArmProfile.kPivotPosThreshold;
    double upperLimit = commandedPivotPos + ArmProfile.kPivotPosThreshold;

    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Checks if arm is below commanded encoder count
   * 
   * @param commandedPivotPos pivot encoder input
   * @return true if arm pivot is below position threshold, otherwise
   */
  public boolean armBelowSetPosition(double commandedPivotPos) {
    double lowerLimit = commandedPivotPos - ArmProfile.kPivotPosThreshold;

    if (pivotEncoderA.getPosition() <= lowerLimit) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean armAboveSetPosition(double commandedPivotPos) {
    double upperLimit = commandedPivotPos + ArmProfile.kPivotPosThreshold;

    if (pivotEncoderA.getPosition() >= upperLimit) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Runs arm to commanded encoder position
   * 
   * @param commandedPivotPos requested encoder pivot position
   * @param upwardOutput positive pivot commanded outout fraction
   * @param downwardOutput negitive pivot commanded output fraction
   */
  public void setArmPivotPos(double commandedPivotPos, double upwardOutputPercent, double downwardOutputPercent) {
    if (armAtSetPosition(commandedPivotPos) == true) {
      setArmOutput(0);
    }
    else if (armBelowSetPosition(commandedPivotPos) == true) {
      setArmOutput(upwardOutputPercent);
    }
    else if (armAboveSetPosition(commandedPivotPos) == true) {
      setArmOutput(-downwardOutputPercent);
    }
  }

  /**
   * Runs arm pivot to commanded encoder position
   * Checks if arm is at commanded encoder position
   * Fires when arm is a commanded encoder position
   * 
   * @param commandedPivotPos requested encoder pivot position
   * @param upwardOutput positive pivot commanded outout fraction
   * @param downwardOutput negitive pivot commanded output fraction
   */
  public void launchNoteAtSetPos(double commandedPos, double upwardOutputPercent, double downwardOutputPercent) {
    setArmPivotPos(commandedPos, upwardOutputPercent, downwardOutputPercent);

    if (armAtSetPosition(commandedPos) == true) {
      setArmOutput(0);
      setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
  }

  /**
   * Sets Shooter & Conveyor Motor to 0
   * Sets arm pivot to initial encoder position
   */
  public void resetArmPivot() {
    setIndexorOuput(0);
    setShooterOutput(0);
    setArmPivotPos(0, 0, 15);
  }

  /**
   * Checks if note is inside the middle of the indexor
   * 
   * @return true if the note breaks beam sensor, otherwise false
   */
  public boolean noteStowed() {
    return beamBreakSensor.get();
  }

  /**
   * Inner intake and conveyor run until note is in the middle of the indexor
   * 
   * @param s_Intake Calls inner subsystem to run inner intake at 100%
   */
  public void stowNote(Intake s_Intake) {
    if (noteStowed() == true) {
      setIndexorOuput(0);
      s_Intake.setInnerRollerOutput(0);
    }
    else {
      setIndexorOuput(30);
      s_Intake.setInnerRollerOutput(100);
    }
  }

  /**
   * Runs arm pivot motors at commanded percentage
   * 
   * @param outputPercent commanded output percentage / 100
   */
  public void setArmOutput(double outputPercent) {
    m_pivotA.set(outputPercent / 100);
    m_pivotB.set(outputPercent / 100);
  }

  /**
   * Runs indexor motor at commanded percentage
   * 
   * @param outputPercent commanded output percentage / 100
   */
  public void setIndexorOuput(double outputPercent) {
    m_indexor.set(outputPercent / 100);
  }

  /**
   * Runs both shooter motors at commanded percentage
   * 
   * @param outputPercent commanded output percentage / 100
   */
  public void setShooterOutput(double outputPercent) {
    m_shooterA.set(outputPercent / 100);
    m_shooterB.set(outputPercent / 100);
  } 

  @Override
  /** This method will be called once per scheduler run */
  public void periodic() {
    /** Encoder Count Readouts */
    SmartDashboard.putNumber("Arm Position (A)", pivotEncoderA.getPosition());
    //SmartDashboard.putNumber("Pivot Position B", pivotEncoderB.getPosition());

    /** Current Readouts */
    SmartDashboard.putNumber("Indexor Current", m_indexor.getSupplyCurrent());
    SmartDashboard.putNumber("Arm Motor Current's ", m_pivotA.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Motor Current's", m_shooterA.getSupplyCurrent());

    /** Sensor Readouts */
    SmartDashboard.putBoolean("Note Is Stored", noteStowed());
    //SmartDashboard.putBoolean("IR Sensor Value", beamBreakSensor.get());
  }
}
