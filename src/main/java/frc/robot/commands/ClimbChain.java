// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

public class ClimbChain extends Command {
  private Climber s_Climber;
  private Arm s_Arm;

  /** Creates a new ClimbChain. */
  public ClimbChain(Climber climber, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Climber = climber;
    s_Arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Climber.isClimberMaxHeight() == true) {
      s_Climber.setClimberToTrueMaxPos();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Climber.setClimberOutput(0);
    s_Arm.setArmPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
