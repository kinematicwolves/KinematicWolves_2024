// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LightShows;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

public class DisabledLightShow extends Command {
  private Lights s_Lights;
  
  /** Creates a new DisableTeleOpLightShow. */
  public DisabledLightShow(Lights s_Lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Lights = s_Lights;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Lights.setTeleOpLightShow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Lights.setDisabledLightShow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
