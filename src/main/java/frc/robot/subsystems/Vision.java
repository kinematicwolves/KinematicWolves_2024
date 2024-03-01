// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightProfile;

public class Vision extends SubsystemBase {
    // Limelight object
    private NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = limeLightTable.getEntry("tx"); // x coordinate
    private NetworkTableEntry ty = limeLightTable.getEntry("ty"); // y coordinate
    private NetworkTableEntry ta = limeLightTable.getEntry("ta"); // Target area
    private NetworkTableEntry tv = limeLightTable.getEntry("tv"); // target valid? boolean
    public boolean limeLightIsOn = false; 

    private NetworkTableEntry LEDModeEntry = limeLightTable.getEntry("ledMode");

    private double[] ffGains = {
      0.0008171388625648901,
      0.0025796090816614394,
      0.004245625441810102,
      0.0028920364306526743,
      -0.004485549864848663,
      -0.017206206747234075,
      -0.027692599432802778,
      -0.022583572720391073,
      0.01028905933557547,
      0.07228314186855418,
      0.14849473849283668,
      0.21195572576869964,
      0.23668096456728935,
      0.21195572576869964,
      0.14849473849283668,
      0.07228314186855418,
      0.01028905933557547,
      -0.022583572720391073,
      -0.027692599432802778,
      -0.017206206747234075,
      -0.004485549864848663,
      0.0028920364306526743,
      0.004245625441810102,
      0.0025796090816614394,
      0.0008171388625648901
  };

  private double[] fbGains = {};

  private LinearFilter filter_d = new LinearFilter(ffGains, fbGains);
  private LinearFilter filter_ha = new LinearFilter(ffGains, fbGains);
  private LinearFilter filter_va = new LinearFilter(ffGains, fbGains);

  private double filtered_distance;
  private double distance;
  private double filtered_h_angle;
  private double h_angle;
  private double filtered_v_angle;
  private double v_angle;

  /** Creates a new Vision2. */
  public Vision() {}

    // Limelight x
  public double getHorizontalAngle() {
    h_angle = tx.getDouble(0.0);
    if (getCaptureStatus() == 1){
      return(h_angle);
    }
    else {
      // If no target found, return a large value so it is not in the alignment window
      return -10;
    }
  }

  // Limelight y
  public double getVerticalAngle() {
    v_angle = ty.getDouble(0.0);
    return(v_angle + LimelightProfile.limelightHeightInches);
  }

  // Limelight area
  public double getTargetArea() {
    double a = ta.getDouble(0.0);
    return(a);
  }

  // Limelight target detected flag
  public double getCaptureStatus() {
    double v = tv.getDouble(0.0);
    return(v);
  }

  // Calculate distance to target
  private double calculateDistance() {
    return (LimelightProfile.speakerHeightInches - LimelightProfile.limelightHeightInches)/Math.sin(Math.toRadians(getFilteredVerticalAngle()));
  }

  // Return distance
  public double getDistance() {
    return(distance);
  }

  // Return filtered horizontal angle
  public double getFilteredHorizontalAngle() {
    h_angle = getHorizontalAngle();
    filtered_h_angle = filter_ha.calculate(h_angle);
    return(filtered_h_angle);
  }

  // Return filtered vertical angle
  public double getFilteredVerticalAngle() {
    v_angle = getVerticalAngle();
    filtered_v_angle = filter_va.calculate(v_angle);
    // 44 is angle from horizontal to limelight aim
    return (filtered_v_angle);
  }

  // Return filtered distance
  public double getFilteredDistance() {
    return(filtered_distance);
  }

  public void turnLimelightOn(){
    LEDModeEntry.setNumber(3);
    limeLightIsOn = true;
  }

  public void turnLimelightOff(){
    LEDModeEntry.setNumber(1);
    limeLightIsOn = false; 
  }

  public boolean isLimeLightOn(){
    return limeLightIsOn;
  }

  public boolean LinedUpWithSpeaker(){
    var horizalAngle = getFilteredHorizontalAngle();
    // degrees
    return (horizalAngle < LimelightProfile.alignWindow) & (horizalAngle > (-1 * LimelightProfile.alignWindow));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance = calculateDistance();
    filtered_distance = filter_d.calculate(distance);

    // SmartDashboard.putNumber("/nLimelight capture status: ", getCaptureStatus());
    //  SmartDashboard.putNumber("LimelightFilteredDistance (inches)", filtered_distance);
    // SmartDashboard.putNumber("LimelightFilteredHorizontalAngle", filtered_h_angle);
    // SmartDashboard.putNumber("LimelightFilteredVerticalAngle", filtered_v_angle);
  }
}
