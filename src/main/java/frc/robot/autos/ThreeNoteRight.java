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
import frc.robot.Constants.SwerveProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Swerve;
 
public class ThreeNoteRight extends SequentialCommandGroup {
    public ThreeNoteRight(Swerve s_Swerve, Arm s_Arm, Intake s_Intake, Lighting s_Lighting){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(SwerveProfile.swerveKinematics);

        Trajectory middleNote =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(0.5, 0)),
          new Pose2d(1.9, 0, new Rotation2d(0.15)),
            config);

        Trajectory leftNote =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(-1.4, -1.5)),
          new Pose2d(0, -2, new Rotation2d(0.0)),
            config);


        Trajectory backUp =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(0.5, 0)),
          new Pose2d(1, 0, new Rotation2d(-0.1)),
            config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                middleNote,
                s_Swerve::getPose,
                SwerveProfile.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

                SwerveControllerCommand swerveControllerCommand2 =
                new SwerveControllerCommand(
                    leftNote,
                    s_Swerve::getPose,
                    SwerveProfile.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);
                
                SwerveControllerCommand swerveControllerCommand3=
                new SwerveControllerCommand(
                    backUp,
                    s_Swerve::getPose,
                    SwerveProfile.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Lighting.setRedLightShow()),
            new TimedShootNote(s_Intake, s_Arm, s_Lighting, ArmProfile.kpivotSpeakerPos, 0.3, 0.15, 1.7),
            new InstantCommand(() -> s_Swerve.setPose(middleNote.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, true, false)),
            new AutoIntakeNote(s_Intake, s_Arm, s_Lighting),
            new TimedShootNote(s_Intake, s_Arm, s_Lighting, 25000, 0.32, 0.15, 2),
            new InstantCommand(() -> s_Swerve.setPose(leftNote.getInitialPose())),
            swerveControllerCommand2,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, true, false)),
            new AutoIntakeNote(s_Intake, s_Arm, s_Lighting),
            new InstantCommand(() -> s_Swerve.setPose(backUp.getInitialPose())),
            swerveControllerCommand3,
            new InstantCommand(() -> s_Lighting.setDisabledLightShow())
        );
    }
}