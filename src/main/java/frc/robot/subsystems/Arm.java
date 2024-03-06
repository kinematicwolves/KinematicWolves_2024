// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

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

  //private WPI_VictorSPX
  private RelativeEncoder pivotEncoderA = m_pivotA.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);
  private RelativeEncoder pivotEncoderB = m_pivotB.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);

  private SparkPIDController pivotControllerA = m_pivotA.getPIDController();
  private SparkPIDController pivotControllerB = m_pivotB.getPIDController();
  
  private double setpoint = 0;

  //private DigitalInput indexorSensor = new DigitalInput(ArmProfile.noteDetectorChannel);

  /** Creates a new Arm. */
  public Arm() {
    /* These are fine to include for testing. However, before comp, save all these settings on the motor drivers, then disable these commands here.
    The failure mode is sometimes these settings do not stick, for whatever reason. I've seen this cause motor driver directions, for example, to not invert correctly.
    These can simply be commented out when running the robot at a competition.
    */
    //TODO: Comment out motor driver settings before comp. 
    //TODO: Use Rev Hardware Client to setup these parameters on the motor driver bfeore comp.
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
    m_pivotA.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_pivotB.enableSoftLimit(SoftLimitDirection.kForward, false);
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

  /* Basic pivot functions */
  /** 
   * Sets the arm PID controller setpoint and moves the arm to that setpoint
   * 
   * @param armSetpoint the setpoint for the arm
   */
  public void setArmPos(double armSetpoint) {
    setpoint = armSetpoint;
    pivotControllerA.setReference(setpoint, ControlType.kPosition);
    m_pivotB.follow(m_pivotA);
  }

  /**
   *  Checks if the arm is near the setpoint
   * 
   * @return true if the arm is at the set point, otherwise false
   */
  public boolean isAtSetpoint() {
    double lowerLimit = setpoint - ArmProfile.kPivotPosThreshold;
    double upperLimit = setpoint + ArmProfile.kPivotPosThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   *  Checks if the arm is below the setpoint
   * 
   * @return true if the arm is below the set point, otherwise false
   */
  public boolean isBelowSetpoint() {
    if (pivotEncoderA.getPosition() <= setpoint) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   *  Checks if the arm is above the setpoint
   * 
   * @return true if the arm is above the set point, otherwise false
   */
  public boolean isAboveSetpoint() {
    if (setpoint <= pivotEncoderA.getPosition()) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   *  Checks if the arm is at a value
   * 
   * @param position the position to check
   * @return true if the arm is at the set point, otherwise false
   */
  public boolean isAtPosition(double position) {
    double lowerLimit = position - ArmProfile.kPivotPosThreshold;
    double upperLimit = position + ArmProfile.kPivotPosThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   *  Checks if the arm is above a value
   * 
   * @param position the position to check
   * @return true if the arm is above the set point, otherwise false
   */
  public boolean isAbovePosition(double position) {
    if (position <= pivotEncoderA.getPosition()) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   *  Checks if the arm is below a value
   * 
   * @param position the position to check
   * @return true if the arm is beleow the set point, otherwise false
   */
  public boolean isBelowPosition(double position) {
    if (pivotEncoderA.getPosition() <= position) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   *  Checks if the arm is above a value
   * 
   * @param commandedOutputFraction the speed to set to the motor
   */
  public void setArmOutput(double commandedOutputFraction) {
    m_pivotA.set(commandedOutputFraction);
    m_pivotB.follow(m_pivotA);
    // m_pivotB.set(commandedOutputFraction);
  }

  /**
   *  Checks if the arm is above a value
   * 
   * @param commandedOutputFraction the speed to set to the motor
   * @return true if the arm is reset, otheerwise
   */
  public boolean isArmReset() {
    return isAtPosition(ArmProfile.pivotInitialPos);
  }

  /* indexor functions */
  public void setIndexorOuput(double commandedOutputFraction) {
    m_indexor.set(commandedOutputFraction);
  }

  /* shooter functions */
  public void setShooterOutput(double commandedOutputFraction) {
    m_shooterA.set(commandedOutputFraction);
    m_shooterB.set(commandedOutputFraction);
  } 

  public void setAmpShooterOutput(double commandedOutputFraction) {
    m_shooterA.set(commandedOutputFraction * 0.2);
    m_shooterB.set(commandedOutputFraction);
  }

  /* assembled functions */
  public void prepareToShoot(Intake s_Intake) {
    setShooterOutput(ArmProfile.kShooterDefaultOutput);
    //if (s_Vision.LinedUpWithSpeaker()) {
      s_Intake.deployPlus();
    //}
  }
  
  public void fireAtSetPos(double commandedPos) {
    //double commandedOutputPos = getPivotPosForDistance(s_Vision.getFilteredDistance());
    setArmPos(commandedPos);
    if (isAtSetpoint()) {
      setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
  }
  
  public void fireAtSpeaker() {
    fireAtSetPos(ArmProfile.kIndexorDefaultOutput);
  }
  
  public void resetArm() {
    setIndexorOuput(0);
    setShooterOutput(0);
    setArmPos(ArmProfile.pivotInitialPos);
  }
  
  public boolean isArmClearForClimb() {
    return isAbovePosition(ArmProfile.kpivotAmpPos);
  }
  
  public void dropNoteInAmp() {
    setArmPos(ArmProfile.kpivotAmpPos);
    if (isAtSetpoint()){
      setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
    else {
      setIndexorOuput(0);
    }
  }
  
  public void setArmToClimbPos() {
    setArmPos(ArmProfile.kpivotAmpPos);
  }
  
  public boolean isNoteDetected() {
    if (m_indexor.getSupplyCurrent() > 4) {
      return true;
    }
    else {
      return false;
    }
  }

  // private double getPivotPosForDistance(double targetdistance){
  //   double requiredPos = LinearInterpolation.linearInterpolation(ArmProfile.TargetDistanceArray, ArmProfile.ArmPosArray, targetdistance);
  //   return requiredPos;
  // }

  // private void setArmFWDSoftLimit() {
  //   if (pivotEncoderA.getPosition() >= ArmProfile.kPivotSoftLiimitFwd) {
  //     setArmPos(ArmProfile.kPivotSoftLiimitFwd - ArmProfile.kPivotPosThreshold);
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setArmFWDSoftLimit();
    isArmReset();

    SmartDashboard.putBoolean("Note Collected", isNoteDetected());
   // SmartDashboard.putBoolean("Arm is Reset", isArmReset());

   SmartDashboard.putNumber("Indexor Current", m_indexor.getSupplyCurrent());
    
    SmartDashboard.putNumber("Arm Position", pivotEncoderA.getPosition());
    SmartDashboard.putNumber("Neo Current (A) ", m_pivotA.getOutputCurrent());
    SmartDashboard.putNumber("Neo Current (B)", m_pivotB.getOutputCurrent());
    SmartDashboard.putNumber("Combined Neo Current", m_pivotA.getOutputCurrent() + m_pivotB.getOutputCurrent());
  }
}
