package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerProfile;
import frc.robot.Constants.SwerveProfile;
import frc.robot.subsystems.Swerve;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier turboModeSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier turboModeSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.turboModeSup = turboModeSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), ControllerProfile.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), ControllerProfile.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), ControllerProfile.stickDeadband);

        /* Slow Speed Drive */
        if (turboModeSup.getAsBoolean() == true) {
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveProfile.maxSpeed), 
                (rotationVal) * SwerveProfile.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true
            );
        }
        /* Normal Speed Drive */
        else {
            s_Swerve.drive(
                new Translation2d(translationVal * SwerveProfile.speedCap, strafeVal * SwerveProfile.speedCap).times(SwerveProfile.maxSpeed), 
                (rotationVal * SwerveProfile.speedCap) * SwerveProfile.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
                );
        }
    }
}