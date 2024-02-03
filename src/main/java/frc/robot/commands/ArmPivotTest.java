// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmPivotTest extends Command {
  private Arm s_Arm;
  private double commandedOutputFraction;

  /** Creates a new JoystickPivotControl. */
  public ArmPivotTest(Arm arm, double commandedOutputFraction) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Arm = arm;
    this.commandedOutputFraction = commandedOutputFraction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.runArmOutput(commandedOutputFraction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}