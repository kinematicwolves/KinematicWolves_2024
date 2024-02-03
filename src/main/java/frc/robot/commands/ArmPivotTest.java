// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerProfile;
import frc.robot.subsystems.Arm;

public class ArmPivotTest extends Command {
  private Arm s_Arm;
  private DoubleSupplier commandedOutputFractionSup;

  /** Creates a new JoystickPivotControl. */
  public ArmPivotTest(Arm arm, DoubleSupplier commandedOutputFractionSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Arm = arm;
    this.commandedOutputFractionSup = commandedOutputFractionSup;
    addRequirements(s_Arm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double commandedOutputFractionVal = MathUtil.applyDeadband(commandedOutputFractionSup.getAsDouble(), ControllerProfile.stickDeadband);
    s_Arm.runArmOutput(commandedOutputFractionVal);
  }
}
