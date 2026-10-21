package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Targets;
import frc.robot.Commands.Intake;
import frc.robot.Commands.Outtake;
import frc.robot.Commands.Shooting;
import frc.robot.Commands.SpinUp;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    // =========================================
    // Subsystems
    // =========================================

    private final DriveSubsystem m_robotDrive =
        new DriveSubsystem();

    private final IntakeSubsystem m_Intake =
        new IntakeSubsystem();

    private final FeederSubsystem m_Feeder =
        new FeederSubsystem();

    private final VisionSubsystem m_Vision =
        new VisionSubsystem();

    // =========================================
    // Controllers
    // =========================================

    private final CommandXboxController m_driverController =
        new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    private final CommandXboxController operatorController =
        new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    // =========================================
    // Auto Chooser
    // =========================================

    private final SendableChooser<Command> autoChooser;

    // =========================================
    // Constructor
    // =========================================

    public RobotContainer() {

        registerNamedCommands();
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Left neutral", AutoBuilder.buildAuto("Left Neutral"));
        autoChooser.addOption("Right neutral", AutoBuilder.buildAuto("Right Neutral"));
        autoChooser.addOption("Middle shoot", AutoBuilder.buildAuto("Middle"));
        autoChooser.addOption("Left", AutoBuilder.buildAuto("Left"));
        autoChooser.addOption("Right", AutoBuilder.buildAuto("Right"));
        autoChooser.addOption("Test", AutoBuilder.buildAuto("Test"));
        autoChooser.addOption("Right Intake", AutoBuilder.buildAuto("Right Intake"));
        autoChooser.addOption("Left Intake", AutoBuilder.buildAuto("Left Intake"));
        autoChooser.addOption("Middle depo", AutoBuilder.buildAuto("Middle Shoot in"));
        autoChooser.addOption("Middle sepo shoot", AutoBuilder.buildAuto("Middle Shoot in shoot"));
        autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("Auto Chooser", autoChooser);


        

        configureButtonBindings();

m_robotDrive.setDefaultCommand(
    new RunCommand(
        () -> m_robotDrive.drive(

            MathUtil.applyDeadband(
                -m_driverController.getLeftY(),
                OperatorConstants.kDriveDeadband),

            MathUtil.applyDeadband(
                -m_driverController.getLeftX(),
                OperatorConstants.kDriveDeadband),

            MathUtil.applyDeadband(
                -m_driverController.getRightX(),
                OperatorConstants.kDriveDeadband)

        ),
        m_robotDrive
    )
);


        SmartDashboard.putData(
            "Auto Chooser",
            autoChooser);
    }

    // =========================================
    // Buttons
    // =========================================

    private void configureButtonBindings() {

        // Lock wheels in X

        m_driverController.rightTrigger()
            .whileTrue(
                new RunCommand(
                    m_robotDrive::setX,
                    m_robotDrive));

        // Zero NavX

        m_driverController.start()
            .onTrue(
                new InstantCommand(
                    m_robotDrive::zeroHeading));


        //Limelight Aim            
        m_driverController.leftTrigger().whileTrue(
    new RunCommand(
        () -> {

            Pose2d target =
                DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red

                        ? Targets.RED_TARGET
                        : Targets.BLUE_TARGET;

            double rot =
                m_robotDrive.getAimRotation(target);

            m_robotDrive.drive(

                MathUtil.applyDeadband(
                    -m_driverController.getLeftY(),
                    OperatorConstants.kDriveDeadband),

                MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    OperatorConstants.kDriveDeadband),

                MathUtil.clamp(
                    rot,
                    -0.7,
                    0.7));

        },
        m_robotDrive
    )
);

        // Intake

        operatorController.leftTrigger()
            .whileTrue(
                new Intake(
                    m_Intake,
                    m_Feeder));

        // Outtake

        operatorController.leftBumper()
            .whileTrue(
                new Outtake(
                    m_Intake,
                    m_Feeder));

        // Feed

        operatorController.rightBumper()
            .whileTrue(

                new SpinUp(
                    m_Intake,
                    IntakeConstants.kFeedNeutralVelocity)

                        .andThen(

                            new Shooting(
                                m_Intake,
                                m_Feeder,
                                IntakeConstants.kFeedNeutralVelocity)));

        

        // Shoot

        operatorController.rightTrigger()
            .whileTrue(

                new SpinUp(
                    m_Intake,
                    IntakeConstants.kShootTowerVelocity)

                        .andThen(

                            new Shooting(
                                m_Intake,
                                m_Feeder,
                                IntakeConstants.kShootTowerVelocity)));

        // Default commands
        

        m_Intake.setDefaultCommand(
            new RunCommand(
                () -> m_Intake.setIntakeMotors(0),
                m_Intake));

        m_Feeder.setDefaultCommand(
            new RunCommand(
                () -> m_Feeder.setFeederMotors(0),
                m_Feeder));
    }

    // =========================================
    // PathPlanner Named Commands
    // =========================================

    private void registerNamedCommands() {

        NamedCommands.registerCommand(
            "Shoot",

            new SpinUp(
                m_Intake,
                IntakeConstants.kShootTowerVelocity)

                    .andThen(

                        new Shooting(
                            m_Intake,
                            m_Feeder,
                            IntakeConstants.kShootTowerVelocity)));

        NamedCommands.registerCommand(
            "Stop Shooting",

            new InstantCommand(
                () -> {

                    m_Intake.setIntakeMotors(0);
                    m_Feeder.setFeederMotors(0);

                },
                m_Intake,
                m_Feeder));

        NamedCommands.registerCommand(
            "Intake",

            new Intake(
                m_Intake,
                m_Feeder));

        NamedCommands.registerCommand(
            "Stop Intake",

            new InstantCommand(
                () -> {

                    m_Intake.setIntakeMotors(0);
                    m_Feeder.setFeederMotors(0);

                },
                m_Intake,
                m_Feeder));

        NamedCommands.registerCommand(
            "Outtake",

            new Outtake(
                m_Intake,
                m_Feeder));

        NamedCommands.registerCommand(
            "Stop Outtake",

            new InstantCommand(
                () -> {

                    m_Intake.setIntakeMotors(0);
                    m_Feeder.setFeederMotors(0);

                },
                m_Intake,
                m_Feeder));
    }

    // =========================================
    // Auto
    // =========================================

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}