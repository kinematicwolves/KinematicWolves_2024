// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoStowNote extends Command {
  private Intake s_Intake;
  private Arm s_Arm;

  /** Creates a new AutoStowNote. */
  public AutoStowNote(Intake intake, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.setIndexorOuput(100);
    s_Intake.setInnerRollerOutput(100);
        if (s_Arm.noteStowed() == true) {
      s_Arm.setIndexorOuput(0);
      s_Intake.setInnerRollerOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Arm.noteStowed() == true;
  }
}
