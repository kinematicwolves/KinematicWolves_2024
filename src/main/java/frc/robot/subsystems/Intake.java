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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDGains;
import frc.robot.Constants.IntakeProfile;

public class Intake extends SubsystemBase {
  /** Wrist Motor */
  private CANSparkMax m_wrist = new CANSparkMax(IntakeProfile.wristID, MotorType.kBrushless);

  /** Wrist Encoder */
  private RelativeEncoder wristEncoder = m_wrist.getEncoder(SparkRelativeEncoder.Type.kHallSensor, IntakeProfile.neoEncoderCountsPerRev);

  /** Wrist Controller */
  private SparkPIDController wristController = m_wrist.getPIDController();

  /** Roller Motors */
  private WPI_TalonSRX m_outerRoller = new WPI_TalonSRX(IntakeProfile.outerRoller);
  private WPI_TalonSRX m_innerRoller = new WPI_TalonSRX(IntakeProfile.innerRoller);
  
  /** Sensors */
  private TimeOfFlight distanceSensor = new TimeOfFlight(0);
  private DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(7);

  /** Initaliaztion Box */
  public Intake() {
    /** Faactory Resets & Fault Clears */
    m_wrist.restoreFactoryDefaults();
    m_outerRoller.configFactoryDefault();
    m_innerRoller.configFactoryDefault();
    m_wrist.clearFaults();

    /** Inversion Factors */
    m_wrist.setInverted(false);
    m_outerRoller.setInverted(true);
    m_innerRoller.setInverted(true);

    /** Current Limits */
    m_wrist.setSmartCurrentLimit(IntakeProfile.kWristCurrentLimit);
    m_outerRoller.configPeakCurrentLimit(IntakeProfile.kOuterRollerCurrentLimit);
    m_innerRoller.configPeakCurrentLimit(IntakeProfile.kInnerRollerCurrentLimit);

    /** Software Limits */
    m_wrist.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_wrist.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_wrist.setSoftLimit(SoftLimitDirection.kReverse, (float)IntakeProfile.kInitialPos);
    m_wrist.setSoftLimit(SoftLimitDirection.kForward, (float)IntakeProfile.kDeployedPos);

    /* Wrist Idle Mode */
    m_wrist.setIdleMode(IdleMode.kBrake);

    /* Wrist Positioning Conversion Factor */
    wristEncoder.setPositionConversionFactor(IntakeProfile.kPositionFactor);
    wristEncoder.setVelocityConversionFactor(IntakeProfile.kVelocityFactor);
    wristEncoder.setPosition(IntakeProfile.kInitialPos);

    /* Wrist Gains */
    PIDGains.setSparkMaxGains(wristController, IntakeProfile.kWristPositionGains);

    /** Flash Wrist Controller Configs */
    m_wrist.burnFlash();

  }

  /**
   * Distance sensor looks for note is inside inner intake
   * 
   * @return true if note is in range, otherwise false
   */
  public boolean noteDetected() {
    if (distanceSensor.getRange() <= IntakeProfile.noteDetectedDistance) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Filtered through bore encoder count position of wrist
   * 
   * @return (encoder count +(-) factory offset error) * (-)1
   */
  public double getFilteredWristPos() {
    return (throughBoreEncoder.getAbsolutePosition() + IntakeProfile.wristPosOffset) * IntakeProfile.wristPosInversion;
  }

  /**
   * Checks if intake is in initial position
   * 
   * @return true if intake position is less than or equal to 1, otherwise false
   */
  public boolean intakePlusUndeployed() {
    if (getFilteredWristPos() <= IntakeProfile.kInitialPos) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Checks if intake is in deployed position
   * 
   * @return true if encoder count is greater than or equal to 9, otherwise false
   */
  public boolean intakePlusDeployed() {
    if (getFilteredWristPos() >= IntakeProfile.kDeployedPos) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Deploys plus plus with no roller output
   */
  public void deployPlus() {
    if (intakePlusDeployed() == true) {
      setWristOutput(0);
      m_wrist.setIdleMode(IdleMode.kCoast);
    }
    else {
      setWristOutput(IntakeProfile.kWristDefaultOutput);
    }
  }

  /**
   * Brings intake plus back up
   * 
   * @param wristOutputFraction output fraction to bring intake back up
   */
  public void undeployPlus(double wristOutputPercent) {
    if (intakePlusUndeployed() == true) {
      setWristOutput(0);
      m_wrist.setIdleMode(IdleMode.kBrake);
    }
    else {
      setWristOutput(-wristOutputPercent);
      m_wrist.setIdleMode(IdleMode.kBrake);
    }
  }

  /**
   * Zero's roller outputs and brings up intake+ at default speed
   * 
   * @param s_Arm calls arm subsytem to zero converyor output
   */
  public void resetIntake(Arm s_Arm, double wristPercentOutput) {
    setInnerRollerOutput(0);
    setOuterRollerOutput(0);
    s_Arm.setIndexorOuput(0);
    undeployPlus(wristPercentOutput);
    }

  /**
   * Intakes with ++ but does not refrence note sensor
   * 
   * @param s_Arm calls arm subsytem for conveyor control
   */
  public void intakePlusPlus(Arm s_Arm) {
    deployPlus();
    if (intakePlusDeployed() == true) {
      setOuterRollerOutput(IntakeProfile.kOuterDefaultOutput);
      setInnerRollerOutput(IntakeProfile.kInnerDefaultOutput);
      s_Arm.setIndexorOuput(50);
    }
  }

  /**
   * Intakes with ++ until note collected
   * 
   * @param s_Arm calls arm subsytem for conveyor control
   * @param s_Lighting calls lighting subsytem for intake status
   */
  // public void smartIntakePlusPlus(Arm s_Arm, Intake s_Intake, Lighting s_Lighting) {
  //   intakePlusPlus(s_Arm);
  //   s_Lighting.setOrangeLightShow();
  //   if (noteDetected() == true) {
  //     resetIntake(s_Arm, IntakeProfile.kWristDefaultOutput);
  //     s_Lighting.setTeleOpLightShow();
  //   }
  // }

  /**
   * Intakes with inner intake only until note collected
   * 
   * @param s_Arm calls arm subsytem for conveyor control
   * @param s_Lighting calls lighting subsytem for intake status
   */
  // public void smartIntakeWithoutPlusPlus(Arm s_Arm, Intake s_Intake, Lighting s_Lighting) {
  //   if (noteDetected() == true) {
  //     undeployPlus(IntakeProfile.kWristDefaultOutput);
  //     s_Arm.stowNote(s_Intake);
  //     s_Lighting.setTeleOpLightShow();
  //   }
  //   else {
  //     intakePlusPlus(s_Arm);
  //     s_Lighting.setOrangeLightShow();
  //   }
  // }

  /**
   * Runs inner roller at a commanded percentage
   * 
   * @param outputPercent commanded output percent / 100
   */
  public void setInnerRollerOutput(double outputPercent) {
    m_innerRoller.set(outputPercent / 100);
  } 

  /**
   * Runs outer roller at a commanded percentage
   * 
   * @param outputPercent commanded output percent / 100
   */
  public void setOuterRollerOutput(double outputPercent) {
    m_outerRoller.set(outputPercent / 100);
  } 

  /**
   * Runs wrist motor at a commanded percentage
   * 
   * @param outputPercent
   */
  public void setWristOutput(double outputPercent) {
    m_wrist.set(outputPercent / 100);
  }

  @Override
  /** This method will be called once per scheduler run */
  public void periodic() {
    /** Current Readouts */
    SmartDashboard.putNumber("Inner Intake Current Output (Amps)", m_innerRoller.getSupplyCurrent());
    SmartDashboard.putNumber("Outer Intkake Current Output (Amps)", m_outerRoller.getSupplyCurrent());
    SmartDashboard.putNumber("Wrist Current Output (Amps)", m_wrist.getOutputCurrent());

    /** Sensor Readouts */
    SmartDashboard.putNumber("Distance Sensor", distanceSensor.getRange());
    SmartDashboard.putBoolean("Note Detected", noteDetected());
    SmartDashboard.putBoolean("Intake+ is Deployed", intakePlusDeployed());
    SmartDashboard.putNumber("Raw Intake Position", throughBoreEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Intake Position", getFilteredWristPos());
  }
}
