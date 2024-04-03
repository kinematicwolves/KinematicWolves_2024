package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmProfile;
import frc.robot.Constants.IntakeProfile;
import frc.robot.Constants.SwerveProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Swerve;
 
public class BlueMidField extends SequentialCommandGroup {
    public BlueMidField(Swerve s_Swerve, Arm s_Arm, Intake s_Intake, Lighting s_Lighting){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(SwerveProfile.swerveKinematics);

        Trajectory midField =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(9, 1)),
          new Pose2d(16.2, 10.8, new Rotation2d(0.17)),
            config);

        Trajectory aim =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(-2, -0.5)),
          new Pose2d(-4, -1.6, new Rotation2d(0.052)),
            config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                midField,
                s_Swerve::getPose,
                SwerveProfile.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

                SwerveControllerCommand swerveControllerCommand2 =
                new SwerveControllerCommand(
                    aim,
                    s_Swerve::getPose,
                    SwerveProfile.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Lighting.setRedLightShow()),
            new TimedShootNote(s_Intake, s_Arm, s_Lighting, 12500, 30, 15, IntakeProfile.kWristDefaultOutput, 1.8),
            new InstantCommand(() -> s_Swerve.setPose(midField.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
            new AutoIntake(s_Intake, s_Arm),
            new InstantCommand(() -> s_Swerve.setPose(aim.getInitialPose())),
            swerveControllerCommand2,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
            new TimedShootNote(s_Intake, s_Arm, s_Lighting, 29000, 35, 15, 11, 3),
            new InstantCommand(() -> s_Lighting.setDisabledLightShow())
        );
    }
}