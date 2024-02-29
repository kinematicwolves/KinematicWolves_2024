// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.TechnitionCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerProfile;
import frc.robot.subsystems.Intake;

public class WristControl extends Command {
  private Intake s_Intake;
  private DoubleSupplier wristSup;
  
  /** Creates a new WristControl. */
  public WristControl(Intake intake, DoubleSupplier wristSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    addRequirements(s_Intake);
    
    this.wristSup = wristSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wristVal = MathUtil.applyDeadband(wristSup.getAsDouble(), ControllerProfile.stickDeadband);
    s_Intake.setWristOutput(wristVal);
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
