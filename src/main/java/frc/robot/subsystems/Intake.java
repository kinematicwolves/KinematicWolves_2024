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

public class Intake extends SubsystemBase {
  private CANSparkMax m_wrist = new CANSparkMax(IntakeProfile.wristID, MotorType.kBrushless);
  private WPI_TalonSRX m_outerRoller = new WPI_TalonSRX(IntakeProfile.outerRoller);
  private WPI_TalonSRX m_innerRoller = new WPI_TalonSRX(IntakeProfile.innerRoller);

  private RelativeEncoder wristEncoder = m_wrist.getEncoder(SparkRelativeEncoder.Type.kHallSensor, IntakeProfile.neoEncoderCountsPerRev);

  private SparkPIDController wristController = m_wrist.getPIDController();
       
  private TimeOfFlight distanceSensor = new TimeOfFlight(0);
  private double safeZoneSensor = distanceSensor.getRange();

  private boolean plusDeployed = false;

  /** Creates a new Intake. */
  public Intake() {
    m_wrist.restoreFactoryDefaults();
    m_outerRoller.configFactoryDefault();
    m_innerRoller.configFactoryDefault();

    m_wrist.clearFaults();

    m_wrist.setInverted(false);
    m_outerRoller.setInverted(true);
    m_innerRoller.setInverted(true);

    m_wrist.setSmartCurrentLimit(IntakeProfile.kWristCurrentLimit);
    m_outerRoller.configPeakCurrentLimit(IntakeProfile.kRollerCurrentLimit);

    m_wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_wrist.setSoftLimit(SoftLimitDirection.kReverse, (float)IntakeProfile.kInitialPos);
    m_wrist.setSoftLimit(SoftLimitDirection.kForward, (float)IntakeProfile.kDeployedPos);

    m_wrist.setIdleMode(IdleMode.kBrake);

    wristEncoder.setPositionConversionFactor(IntakeProfile.kPositionFactor);
    wristEncoder.setVelocityConversionFactor(IntakeProfile.kVelocityFactor);
    wristEncoder.setPosition(IntakeProfile.kInitialPos);

    PIDGains.setSparkMaxGains(wristController, IntakeProfile.kWristPositionGains);

    m_wrist.burnFlash();
  }

  public void deployPlus() {
    if (wristEncoder.getPosition() >= IntakeProfile.kDeployedLowerLimitPos) {
      setWristOutput(0);
      m_wrist.setIdleMode(IdleMode.kCoast);
      plusDeployed = true;
    }
    else {
      m_wrist.set(0.2);
    }
  }

  private void deployAndIntake(Arm s_Arm) {
    deployPlus();
    if (wristEncoder.getPosition() >= IntakeProfile.kDeployedLowerLimitPos) {
      setOuterRollerOutput(IntakeProfile.kOuterDefaultOutput);
      setInnerRollerOutput(IntakeProfile.kInnerDefaultOutput);
      s_Arm.setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
    }
    // else if (safeZoneSensor <= IntakeProfile.kPlusSafeZone) {
    //   IntakeButHoldThePlus(s_Arm);
    // }
  }

  private void IntakeButHoldThePlus(Arm s_Arm) {
    if (wristEncoder.getPosition() <= IntakeProfile.kInitailUpperLimitPos) {
      setWristOutput(0);
      setInnerRollerOutput(IntakeProfile.kInnerDefaultOutput);
      s_Arm.setIndexorOuput(ArmProfile.kIndexorDefaultOutput);
      m_wrist.setIdleMode(IdleMode.kBrake);
      plusDeployed = false;
    }
    else if (wristEncoder.getPosition() >= IntakeProfile.kInitailUpperLimitPos) {
      setWristOutput(-0.1);
    }
    else {
      m_wrist.set(-0.8);
      setOuterRollerOutput(0);
    }
  }

  public void resetIntake(Arm s_Arm) {
    setInnerRollerOutput(0);
    setOuterRollerOutput(0);
    s_Arm.setIndexorOuput(0);
    if (wristEncoder.getPosition() <= IntakeProfile.kInitailUpperLimitPos) {
      setWristOutput(0);
      m_wrist.setIdleMode(IdleMode.kBrake);
      plusDeployed = false;
    }
    else if (wristEncoder.getPosition() >= IntakeProfile.kInitailUpperLimitPos) {
      setWristOutput(-0.1);
    }
    else {
      m_wrist.set(-0.25);
    }
  }

  public void enableIntake(Arm s_Arm) {
    // if (s_Arm.isNoteDetected() == true) {
    //   resetIntake(s_Arm);
    // }
    //else {
      deployAndIntake(s_Arm);
    //}
  }

  public boolean isIntakePlusEnabled() {
    return plusDeployed;
  }

  public void setInnerRollerOutput(double commandedOutputFraction) {
    m_innerRoller.set(commandedOutputFraction);
  } 

  public void setOuterRollerOutput(double commandedOutputFraction) {
    m_outerRoller.set(commandedOutputFraction);
  } 

  public void setWristOutput(double commandedOutputFraction) {
    m_wrist.set(commandedOutputFraction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Inner Intake Current Output (Amps)", m_innerRoller.getSupplyCurrent());
    SmartDashboard.putNumber("Outer Intkake Current Output (Amps)", m_outerRoller.getSupplyCurrent());
    SmartDashboard.putNumber("Wrist Current Output (Amps)", m_wrist.getOutputCurrent());
    SmartDashboard.putBoolean("Intake++ is Deployed", isIntakePlusEnabled());
    SmartDashboard.putNumber("Wrist Encoder Counts", wristEncoder.getPosition());
  }
}
