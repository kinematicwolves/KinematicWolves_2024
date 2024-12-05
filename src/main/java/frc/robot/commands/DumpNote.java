// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmProfile;
import frc.robot.Constants.IntakeProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class DumpNote extends Command {
  private Intake s_Intake;
  private Arm s_Arm;

  /** Creates a new DropAmpNote. */
  public DumpNote(Intake intake, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.prepareToDump(s_Intake);
    if (s_Intake.intakePlusDeployed() == true) {
      s_Arm.launchNoteAtSetPos(ArmProfile.kpivotAmpPos, 50, 15);}
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.resetArmPivot(39);
    s_Intake.resetIntake(s_Arm, IntakeProfile.kWristSlowOutput);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
