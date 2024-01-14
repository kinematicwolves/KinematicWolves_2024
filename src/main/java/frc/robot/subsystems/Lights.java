// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightingProfile;

public class Lights extends SubsystemBase {
  private CANdle candle;
  private CANdleConfiguration cfg;
  private Animation animation;

  /** Creates a new Lighting. */
  public Lights() {
    candle = new CANdle(0, "canivore1");
    candle.configAllSettings(cfg);
    candle.configLEDType(LEDStripType.RGB);

    cfg = new CANdleConfiguration();
    cfg.brightnessScalar = LightingProfile.kBrightnessScalar;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;

    animation = null;
  }

  public void setTeleOpLightShow() {
    animation = new ColorFlowAnimation(0, 225, 0, 100, 1, LightingProfile.numLEDStrip, Direction.Forward);
  }

  public void setDisabledLightShow() {
    animation = new ColorFlowAnimation(0, 0, 0, 100, 1, LightingProfile.numLEDStrip, Direction.Forward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (animation != null){
      candle.animate(animation);
    }
  }
}
