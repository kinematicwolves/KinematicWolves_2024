// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmProfile;
import frc.robot.Constants.ClimberProfile;
import frc.robot.Constants.ControllerProfile;
import frc.robot.Constants.IntakeProfile;
import frc.robot.RobotStates.SetDisabledState;
import frc.robot.RobotStates.SetEnabledState;
import frc.robot.autos.AmpAuto;
import frc.robot.autos.AutoIntakePlus;
import frc.robot.autos.BackUp;
import frc.robot.autos.OneNoteLeft;
import frc.robot.autos.OneNoteRight;
// import frc.robot.TechnitionCommands.ArmControl;
// import frc.robot.TechnitionCommands.WristControl;
import frc.robot.autos.FourNote;
import frc.robot.autos.BlueMidField;
import frc.robot.autos.BlueOneNoteRight;
import frc.robot.autos.TwoNoteRight;
import frc.robot.commands.ClimbChain;
import frc.robot.commands.DumpNote;
import frc.robot.commands.IntakeFromSource;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.PreClimbState;
import frc.robot.commands.ShootNote;
import frc.robot.commands.ShootTrap;
import frc.robot.commands.StowNote;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(ControllerProfile.kDriverControllerPort);
    private final Joystick munipulator = new Joystick(ControllerProfile.kManipulatorControllerPort);
    // private final Joystick technition = new Joystick(ControllerProfile.kTechnitionControllerPort);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
    /* Technition Controls */
    // private final int wristAxis = XboxController.Axis.kRightY.value;
    // private final int armAxis = XboxController.Axis.kLeftY.value;

    /* Sendable Choosers */
    SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm s_Arm = new Arm();
    private final Intake s_Intake = new Intake();
    private final Climber s_Climber = new Climber();
    private final Lighting s_Lighting = new Lighting();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //Configure the button bindings
        configureButtonBindings();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> slowMode.getAsBoolean()
            )
        );

        /* Uncomment for technition wrist control */
        // s_Intake.setDefaultCommand(new WristControl(
        //     s_Intake,
        //     () -> -technition.getRawAxis(wristAxis)));
            
        /* Uncomment for technition arm control */
        // s_Arm.setDefaultCommand(new ArmControl(
        //     s_Arm, 
        //     () -> -technition.getRawAxis(armAxis)));

        /* Chooser for Auton Commands */
        m_AutoChooser.setDefaultOption("BackUp", new BackUp(s_Swerve, s_Arm, s_Intake, s_Lighting));
        m_AutoChooser.addOption("4 Note", new FourNote(s_Swerve, s_Arm, s_Intake, s_Lighting));
        m_AutoChooser.addOption("Blue 1 Note Mid-Field", new BlueMidField(s_Swerve, s_Arm, s_Intake, s_Lighting));
        m_AutoChooser.addOption("Blue 1 Note Taxi", new BlueOneNoteRight(s_Swerve, s_Arm, s_Intake, s_Lighting));
        // m_AutoChooser.addOption("2 Note Right", new TwoNoteRight(s_Swerve, s_Arm, s_Intake, s_Lighting));
        // m_AutoChooser.addOption("1 Note Red", new OneNoteLeft(s_Swerve, s_Arm, s_Intake, s_Lighting));
        // m_AutoChooser.addOption("1 Note Blue", new OneNoteRight(s_Swerve, s_Arm, s_Intake, s_Lighting));
        // m_AutoChooser.addOption("Shoot And Amp", new AmpAuto(s_Swerve, s_Arm, s_Intake, s_Lighting));
        SmartDashboard.putData(m_AutoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        new JoystickButton(driver, XboxController.Button.kY.value) // Y = Zero Gryo
        .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /* Manipulator Buttons */
        new JoystickButton(munipulator, XboxController.Button.kA.value) // A = Intake 
        .whileTrue(new IntakeNote(s_Intake, s_Arm, s_Lighting));
        new JoystickButton(munipulator, XboxController.Button.kA.value)
        .onFalse(new StowNote(s_Arm, s_Intake)); // A = Intake 
        new JoystickButton(munipulator, XboxController.Button.kLeftBumper.value) // LB = Shoot Trap
        .whileTrue(new ShootTrap(s_Swerve, s_Intake, s_Arm));
        new JoystickButton(munipulator, XboxController.Button.kRightBumper.value) // RB = Source Intake
        .whileTrue(new IntakeFromSource(s_Arm, s_Intake));
        new JoystickButton(munipulator, XboxController.Button.kY.value) // Y = Shoot At Speaker
        .whileTrue(new ShootNote(s_Swerve, s_Intake, s_Arm, s_Lighting));
        new JoystickButton(munipulator, XboxController.Button.kX.value) // X = Dump In Amp
        .whileTrue(new DumpNote(s_Intake, s_Arm));
        new JoystickButton(munipulator, XboxController.Button.kBack.value) // Back = Climbers to First State
        .whileTrue(new PreClimbState(s_Intake, s_Arm, s_Climber)); 
        new JoystickButton(munipulator, XboxController.Button.kStart.value) // Start = Climb Chain
        .whileTrue(new ClimbChain(s_Climber, s_Arm));
        new JoystickButton(munipulator, XboxController.Button.kLeftStick.value) // Down Left Stick = Climber A
        .onTrue(new InstantCommand(() -> s_Climber.setClimberAOutput(ClimberProfile.outputWithRobotLoad)))
        .onFalse(new InstantCommand(() -> s_Climber.setClimberAOutput(0)));
        new JoystickButton(munipulator, XboxController.Button.kRightStick.value) // Down Right Stick = Climber B
        .onTrue(new InstantCommand(() -> s_Climber.setClimberBOutput(ClimberProfile.outputWithRobotLoad)))
        .onFalse(new InstantCommand(() -> s_Climber.setClimberBOutput(0)));

        /* Technition Buttons */
        // new JoystickButton(technition, XboxController.Button.kA.value) // X = Inner Roller
        // .onTrue(new InstantCommand(() -> s_Intake.setInnerRollerOutput(IntakeProfile.kInnerDefaultOutput)))
        // .onTrue(new InstantCommand(() -> s_Arm.setIndexorOuput(ArmProfile.kIndexorDefaultOutput)))
        // .onFalse(new InstantCommand(() -> s_Intake.setInnerRollerOutput(0)))
        // .onFalse(new InstantCommand(() -> s_Arm.setIndexorOuput(0)));
        // new JoystickButton(technition, XboxController.Button.kB.value) // B = Outer Roller
        // .onTrue(new InstantCommand(() -> s_Intake.setOuterRollerOutput(IntakeProfile.kOuterDefaultOutput)))
        // .onFalse(new InstantCommand(() -> s_Intake.setOuterRollerOutput(0)));
        // new JoystickButton(technition, XboxController.Button.kLeftBumper.value) // LB = Indexor
        // .onTrue(new InstantCommand(() -> s_Arm.setIndexorOuput(ArmProfile.kIndexorDefaultOutput)))
        // .onFalse(new InstantCommand(() -> s_Arm.setIndexorOuput(0)));
        // new JoystickButton(technition, XboxController.Button.kRightBumper.value) // RB = Shooter
        // .onTrue(new InstantCommand(() -> s_Arm.setShooterOutput(ArmProfile.kShooterAmpOutput)))
        // .onFalse(new InstantCommand(() -> s_Arm.setShooterOutput(0)));
        // new JoystickButton(technition, XboxController.Button.kY.value) // Y = Climbers
        // .onTrue(new AutoIntakeNote(s_Intake, s_Arm, s_Lighting));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_AutoChooser.getSelected();
    }

    // Command to reset robot to initial teleop lightshow/state
    public Command getTeleOpInitCommand() {
        return new SetEnabledState(s_Lighting);     
    }

    // Command to reset robot to initial sisabled lightshow/state
    public Command getDisabledCommandInitCommand() {
        return new SetDisabledState(s_Lighting);
    }
}
