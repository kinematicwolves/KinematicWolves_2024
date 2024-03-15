// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightingProfile;

public class Lighting extends SubsystemBase {
  private CANdle candle = new CANdle(LightingProfile.candldeID);
  private Animation animation = null;

  /** Creates a new Lighting. */
  public Lighting() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.brightnessScalar = LightingProfile.kBrightnessScalar;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;

    candle.configAllSettings(cfg);
    candle.configLEDType(LEDStripType.GRB);
    setDisabledLightShow();
  }

  public void setTeleOpLightShow() {
    animation = new TwinkleAnimation(0, 225, 10, 100, 1, LightingProfile.numLEDStrip, TwinklePercent.Percent100);
    //animation = new ColorFlowAnimation(0, 225, 10, 100, 0.8, LightingProfile.numLEDStrip, Direction.Forward);
  }

  public void setDisabledLightShow() {
    animation = new RainbowAnimation(0.1, 0.6, LightingProfile.numLEDStrip);
    //animation = new TwinkleAnimation(0, 0, 0, 225, 1, LightingProfile.numLEDStrip, TwinklePercent.Percent100);
  }

  public void setTestLightShow() {
    //animation = new TwinkleAnimation(225, 140, 0, 0, 1, LightingProfile.numLEDStrip, TwinklePercent.Percent100);
    animation = new FireAnimation(1, 0.2, LightingProfile.numLEDStrip, 0.1, 0.1);
  }

  public void setOrangeLightShow() {
    animation = new TwinkleAnimation(225, 140, 0, 0, 1, LightingProfile.numLEDStrip, TwinklePercent.Percent100);
  }

  public void setRedLightShow() {
    animation = new TwinkleAnimation(225, 0, 0, 5, 1, LightingProfile.numLEDStrip, TwinklePercent.Percent100);
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    if (animation != null){
      candle.animate(animation);
    }
  }
}