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
import frc.robot.Constants.SwerveProfile;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Swerve;
 
public class DriveFWD extends SequentialCommandGroup {
    public DriveFWD(Swerve s_Swerve, Arm s_Arm, Intake s_Intake, Lighting s_Lighting){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(SwerveProfile.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory backupTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
              List.of(new Translation2d(7, 0)), //<- change x, y to go to a different posiiton
              new Pose2d(3, 0, new Rotation2d(0.04)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                backupTrajectory,
                s_Swerve::getPose,
                SwerveProfile.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Lighting.setRedLightShow()),
            new InstantCommand(() -> s_Swerve.setPose(backupTrajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> s_Swerve.drive(new Translation2d(0,0), 0, true, false)),
            new InstantCommand(() -> s_Lighting.setDisabledLightShow())
        );
    }
}