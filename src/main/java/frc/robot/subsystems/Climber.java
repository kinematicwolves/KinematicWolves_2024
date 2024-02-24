// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberProfile;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
  private TalonFX m_climberA = new TalonFX(ClimberProfile.climberA_ID);
  private TalonFX m_climberB = new TalonFX(ClimberProfile.climberB_ID);

  /** Creates a new Climber. */
  public Climber() {
    m_climberA.getConfigurator().apply(Robot.ctreConfigs.climberFXConfig);
    m_climberB.getConfigurator().apply(Robot.ctreConfigs.climberFXConfig);

    m_climberA.setInverted(false); 
    m_climberB.setInverted(true);
  }

  public void setClimberOutput(double commandedOutputFraction) {
    m_climberA.set(commandedOutputFraction);
    m_climberB.set(commandedOutputFraction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
