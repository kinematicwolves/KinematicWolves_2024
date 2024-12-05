// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class PreClimbState extends Command {
  private Intake s_Intake;
  private Arm s_Arm;
  private Climber s_Climber;

  /** Creates a new RunClimbersToFirstState. */
  public PreClimbState(Intake intake, Arm arm, Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
    s_Climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.deployPlus();
    s_Climber.setClimberToClimbPos();
    if (s_Intake.intakePlusDeployed() == true) {
      s_Arm.setArmPivotPos(ArmProfile.kpivotAmpPos, 38, 15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Climber.setClimberOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
