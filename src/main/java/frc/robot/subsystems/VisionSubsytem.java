// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.InputStreamReader;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsytem extends SubsystemBase {
  private PhotonCamera camera = new PhotonCamera("ArduCam");
  // private UsbCamera usbCam = new UsbCamera("Camera", 1);
  // MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);


  /** Creates a new VisionSubsytem. */
  public VisionSubsytem() {
    camera.setDriverMode(true);
    // usbCam.getVideoMode();
    // mjpegServer1.setSource(usbCam);
    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
