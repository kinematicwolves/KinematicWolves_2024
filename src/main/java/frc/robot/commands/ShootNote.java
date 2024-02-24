// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class ShootNote extends Command {
  private Swerve s_Swerve;
  private Vision s_Vision;
  private Intake s_Intake;
  private Arm s_Arm;

  private int timer;

  /** Creates a new ShootNote. */
  public ShootNote(Swerve swerve, Vision vision, Intake intake, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = swerve;
    s_Vision = vision;
    s_Intake = intake;
    s_Arm = arm;

    timer = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;

    //s_Swerve.rotateDrivetrainToTarget(s_Vision);
    s_Arm.prepareToShoot(s_Swerve, s_Vision, s_Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 20;

    if (timer > 1000) {
    s_Arm.fireAtTarget(s_Vision);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.setArmPos(ArmProfile.pivotInitialPos);
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
