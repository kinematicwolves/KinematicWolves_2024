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

  //private DigitalInput indexorSensor = new DigitalInput(ArmProfile.noteDetectorChannel);

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

  /* Set Arm To Position Logic:
   * When arm is at commanded position:
   *    -Set arm to 0 (brake mode will hold it in place)
   * When arm is higher then commanded position:
   *    -Set arm output to -12%
   * When are is lower then commanded position:
   *    -Set arm output to 50%
   */
  public void setArmPos() {
    double lowerLimit = 50000 - 800;
    double upperLimit = 50000 + 600;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      setArmOutput(0);
    }
    else if (pivotEncoderA.getPosition() <= lowerLimit) {
      setArmOutput(0.38);
    }
    else {
      setArmOutput(-0.18);
    }
  }

  /* Prepare Robot to Shoot Function Logic:
   * -Set shooter motors to 100%
   * -Deploy Intake Plus Plus
   */
  public void prepareToShoot(Intake s_Intake) {
    setShooterOutput(ArmProfile.kShooterDefaultOutput);
    //if (s_Vision.LinedUpWithSpeaker()) {
      s_Intake.deployPlus();
    //}
  }

  // private double getPivotPosForDistance(double targetdistance){
  //   double requiredPos = LinearInterpolation.linearInterpolation(ArmProfile.TargetDistanceArray, ArmProfile.ArmPosArray, targetdistance);
  //   return requiredPos;
  // }

  public void fireAtSpeaker() {
    //double commandedOutputPos = getPivotPosForDistance(s_Vision.getFilteredDistance());
    double lowerLimit = ArmProfile.kpivotSpeakerPos - ArmProfile.kPivotPosThreshold;
    double upperLimit = ArmProfile.kpivotSpeakerPos + ArmProfile.kPivotPosThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      setArmOutput(0);
      setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
    else if (pivotEncoderA.getPosition() <= lowerLimit) {
      setArmOutput(0.33);
    }
    else {
      setArmOutput(-0.15); // 20% negitive output
    }
  }

  public void fireAtSetPos(double commandedPos) {
    //double commandedOutputPos = getPivotPosForDistance(s_Vision.getFilteredDistance());
    double lowerLimit = commandedPos - ArmProfile.kPivotPosThreshold;
    double upperLimit = commandedPos + ArmProfile.kPivotPosThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      setArmOutput(0);
      setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
    else if (pivotEncoderA.getPosition() <= lowerLimit) {
      setArmOutput(0.4);
    }
    else {
      setArmOutput(-0.16);
    }
  }


  public void resetArm() {
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

  public boolean isArmReset() {
    if (pivotEncoderA.getPosition() <= ArmProfile.pivotInitialPos + ArmProfile.kPivotPosThreshold) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isArmClearForClimb() {
    if (pivotEncoderA.getPosition() >= ArmProfile.kpivotAmpPos - ArmProfile.kPivotPosThreshold) {
      return true;
    }
    else {
      return false;
    }
  }

  public void dropNoteInAmp() {
    double lowerLimit = ArmProfile.kpivotAmpPos - ArmProfile.kPivotPosThreshold;
    double upperLimit = ArmProfile.kpivotAmpPos + ArmProfile.kPivotPosThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      setArmOutput(0);
      setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
    else if (pivotEncoderA.getPosition() <= lowerLimit) {
      setArmOutput(0.5);
    }
    else {
      setArmOutput(-0.15);
    }
  }

  public void setArmToClimbPos() {
    double lowerLimit = ArmProfile.kpivotAmpPos - ArmProfile.kPivotPosThreshold;
    double upperLimit = ArmProfile.kpivotAmpPos + ArmProfile.kPivotPosThreshold;
    if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit)) {
      setArmOutput(0);
    }
    else if (pivotEncoderA.getPosition() <= lowerLimit) {
      setArmOutput(0.35);
    }
    else {
      setArmOutput(-0.15);
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

  public void setAmpShooterOutput(double commandedOutputFraction) {
    m_shooterA.set(commandedOutputFraction * 0.2);
    m_shooterB.set(commandedOutputFraction);
  }

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

   // SmartDashboard.putBoolean("Note Collected", isNoteDetected());
   // SmartDashboard.putBoolean("Arm is Reset", isArmReset());

   SmartDashboard.putNumber("Indexor Current", m_indexor.getSupplyCurrent());
    
    SmartDashboard.putNumber("Arm Position", pivotEncoderA.getPosition());
    SmartDashboard.putNumber("Neo Current (A) ", m_pivotA.getOutputCurrent());
    SmartDashboard.putNumber("Neo Current (B)", m_pivotB.getOutputCurrent());
    SmartDashboard.putNumber("Combined Neo Current", m_pivotA.getOutputCurrent() + m_pivotB.getOutputCurrent());
  }
}
