// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;

public class TimedShootNote extends Command {
  /** Creates a new TimedShootNote. */
  private Intake s_Intake;
  private Arm s_Arm;
  private Lighting s_Lighting;
  private double pivotAngle;
  private double seconds;
  private int timer;

  public TimedShootNote(Intake intake, Arm arm, Lighting lighting, double pivotAngle, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Arm = arm;
    s_Lighting = lighting;
    this.pivotAngle = pivotAngle;
    this.seconds = seconds;
    timer = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    s_Arm.prepareToShoot(s_Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 20;
    if (s_Intake.isIntakePlusEnabled() == true) {
      s_Arm.fireAtSetPos(pivotAngle);
      }
    // if (timer >= Units.secondsToMilliseconds(seconds)) {
    //   s_Arm.resetArm();
    //   s_Intake.resetIntake(s_Arm, s_Lighting);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.resetArm();
    s_Intake.resetIntake(s_Arm, s_Lighting);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer >= Units.secondsToMilliseconds(seconds);
  }
}
