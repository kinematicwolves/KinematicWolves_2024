// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerProfile;
import frc.robot.subsystems.Intake;

public class WristControlTest extends Command {
  /** Creates a new WristControlTest. */
  private Intake s_Intake;
  private DoubleSupplier pivotSup;

  public WristControlTest(Intake intake, DoubleSupplier pivotSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    this.pivotSup = pivotSup;
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pivotVal = MathUtil.applyDeadband(pivotSup.getAsDouble(), ControllerProfile.stickDeadband);
    s_Intake.setWristOutput(pivotVal);
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
