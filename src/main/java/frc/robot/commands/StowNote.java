// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;

public class StowNote extends Command {
  private Arm s_Arm;
  private Intake s_Intake;
  private Lighting s_Lighting;
  
  /** Creates a new StowNote. */
  public StowNote(Arm arm, Intake intake, Lighting lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Lighting = lighting;
    s_Arm = arm;
    //addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // s_Lighting.setRedLightShow();
    // s_Arm.setIndexorOuput(65);
    // System.out.println("stow command initialization");
    // s_Intake.setInnerRollerOutput(100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Intake.noteDetected() == true) {
      //s_Lighting.setRedLightShow();
      s_Arm.setIndexorOuput(65);
      //System.out.println("run motors");
      s_Intake.setInnerRollerOutput(100);
    }
    // else if (s_Arm.noteStowed() == true) {
    //   s_Lighting.setRedLightShow();
    //   s_Arm.setIndexorOuput(0);
    //   System.out.println("note is in");
    //   s_Intake.setInnerRollerOutput(0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("end of stow command");
    s_Arm.setIndexorOuput(0);
    s_Lighting.setTeleOpLightShow();
    s_Intake.setInnerRollerOutput(0);
    s_Intake.resetIntake(s_Arm, IntakeProfile.kWristDefaultOutput);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("End of stow command");
    return s_Arm.noteStowed() == true;
  }
}
