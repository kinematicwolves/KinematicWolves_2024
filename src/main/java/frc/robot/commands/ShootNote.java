// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmProfile;
import frc.robot.Constants.IntakeProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Swerve;

public class ShootNote extends Command {
  private Intake s_Intake;
  private Arm s_Arm;
  private Lighting s_Lighting;

  /** Creates a new ShootNote. */
  public ShootNote(Swerve swerve, Intake intake, Arm arm, Lighting lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
    s_Lighting = lighting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //s_Swerve.rotateDrivetrainToTarget(s_Vision);
    s_Arm.prepareToShoot(s_Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Intake.intakePlusDeployed() == true) {
    s_Arm.launchNoteAtSetPos(ArmProfile.kpivotSpeakerPos, 30, 15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.resetArmPivot(15);
    s_Intake.resetIntake(s_Arm, IntakeProfile.kWristDefaultOutput);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
