// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;

public class TimedShootNote extends Command {
  /** Creates a new TimedShootNote. */
  private Intake s_Intake;
  private Arm s_Arm;
  private Lighting s_Lighting;
  private double pivotAngle;
  private double upwardOutput;
  private double downwardOutput;
  private double seconds;
  private double wristResetSpeed;
  private int timer;

  public TimedShootNote(Intake intake, Arm arm, Lighting lighting, double pivotAngle, double upwardOutput, double downwardOutput, double wristResetSpeed, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
    s_Lighting = lighting;
    this.pivotAngle = pivotAngle;
    this.upwardOutput = upwardOutput;
    this.downwardOutput = downwardOutput;
    this.wristResetSpeed = wristResetSpeed;
    this.seconds = seconds;
    timer = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 20;
    s_Arm.prepareToShoot(s_Intake);
    //if (s_Intake.intakePlusDeployed() == true) {
      s_Arm.launchNoteAtSetPos(pivotAngle, upwardOutput, downwardOutput);
      //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.resetArmPivot(25);
    s_Intake.resetIntake(s_Arm, wristResetSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer >= Units.secondsToMilliseconds(seconds);
  }
}
