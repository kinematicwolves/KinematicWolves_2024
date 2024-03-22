// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;

public class DumpNote extends Command {
  private Intake s_Intake;
  private Arm s_Arm;
  private Lighting s_Lighting;

  /** Creates a new DropAmpNote. */
  public DumpNote(Intake intake, Arm arm, Lighting lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
    s_Lighting = lighting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Arm.prepareToDump(s_Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Intake.isIntakePlusEnabled() == true) {
      s_Arm.launchAtSetPos(ArmProfile.kpivotAmpPos, 0.5, 0.15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.resetArmPivot();
    s_Intake.resetIntakeForAmp(s_Arm, s_Lighting);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
