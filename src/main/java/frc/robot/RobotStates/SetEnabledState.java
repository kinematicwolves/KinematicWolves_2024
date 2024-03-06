// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotStates;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lighting;

public class SetEnabledState extends Command {
  private Lighting s_Lighting;
  private int matchTimer;

  /** Creates a new TeleOpLightShow. */
  public SetEnabledState(Lighting lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Lighting = lighting;
    matchTimer = 120000;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    matchTimer = 120000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    matchTimer -= 20;
    s_Lighting.setTeleOpLightShow();
    SmartDashboard.putNumber("Match Time Remaining", Units.millisecondsToSeconds(matchTimer));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Lighting.setDisabledLightShow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}