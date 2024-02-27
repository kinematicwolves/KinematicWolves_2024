// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class DropAmpNote extends Command {
  private Intake s_Intake;
  private Arm s_Arm;

  /** Creates a new DropAmpNote. */
  public DropAmpNote(Intake intake, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Intake.deployPlus();
    s_Arm.setArmOutput(ArmProfile.kShooterAmpOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Intake.isIntakePlusEnabled() == true) {
      s_Arm.dropNoteInAmp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.resetArm();
    if (s_Arm.isArmReset() == true) {
      s_Intake.resetIntake(s_Arm);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
