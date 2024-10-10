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
import frc.robot.autos.AutoCommands.AutoIntakePlus;
import frc.robot.autos.AutoCommands.AutoStowNote;
import frc.robot.autos.AutoCommands.TimedShootNote;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Swerve;
 
public class CenterLeftAuto extends SequentialCommandGroup {
    public CenterLeftAuto (Swerve s_Swerve, Arm s_Arm, Intake s_Intake, Lighting s_Lighting){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(SwerveProfile.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory leftNote =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
              List.of(new Translation2d(0.9, 1.6)),
              new Pose2d(1.5, 2.55, new Rotation2d(-0.31)),  
            
                config);

        Trajectory middleNote =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
              List.of(new Translation2d(-1, -0.8)),
              new Pose2d(-0.08, -2.49, new Rotation2d(0.13)),
                config);

        Trajectory rightNote =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
              List.of(new Translation2d(-0.4, -0.5)),
              new Pose2d(-0.08, -2.25, new Rotation2d(0.37)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                    leftNote,
                    s_Swerve::getPose,
                    SwerveProfile.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

                    SwerveControllerCommand swerveControllerCommand2 =
                new SwerveControllerCommand(
                    middleNote,
                    s_Swerve::getPose,
                    SwerveProfile.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

                    
                    SwerveControllerCommand swerveControllerCommand3 =
                new SwerveControllerCommand(
                    rightNote,
                    s_Swerve::getPose,
                    SwerveProfile.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Lighting.setRedLightShow()),
            new TimedShootNote(s_Intake, s_Arm, s_Lighting, ArmProfile.kpivotSpeakerPos, 30, 16, IntakeProfile.kWristDefaultOutput, 1.8),
            new InstantCommand(() -> s_Swerve.setPose(leftNote.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
            new AutoIntakePlus(s_Intake, s_Arm, s_Lighting),
            new AutoStowNote(s_Intake, s_Arm),
            new InstantCommand(() -> s_Swerve.setPose(middleNote.getInitialPose())),
            swerveControllerCommand2,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
            new TimedShootNote(s_Intake, s_Arm, s_Lighting, 21900, 30, 15, IntakeProfile.kWristDefaultOutput, 1.9),
            new AutoIntakePlus(s_Intake, s_Arm, s_Lighting),
            new AutoStowNote(s_Intake, s_Arm),
            new TimedShootNote(s_Intake, s_Arm, s_Lighting, 21900, 30, 15, IntakeProfile.kWristDefaultOutput, 1.9),
            new InstantCommand(() -> s_Swerve.setPose(rightNote.getInitialPose())),
            swerveControllerCommand3,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, true)),
            new InstantCommand(() -> s_Lighting.setDisabledLightShow())
        );
    }
}