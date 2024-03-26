// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;

public class AutoIntakeNote extends Command {
  /** Creates a new TimedShootNote. */
  private Intake s_Intake;
  private Arm s_Arm;
  private Lighting s_Lighting;

  public AutoIntakeNote(Intake intake, Arm arm, Lighting lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
    s_Lighting = lighting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Intake.noteDetected() == true) {
      s_Intake.undeployPlus(IntakeProfile.kWristDefaultOutput);
      s_Arm.stowNote(s_Intake, s_Lighting);
    }
    else {
      s_Intake.smartIntakePlusPlus(s_Arm, s_Intake, s_Lighting);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Intake.noteDetected() == true);
  }
}
