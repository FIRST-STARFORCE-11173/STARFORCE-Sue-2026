package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.Commands.Intake;
import frc.robot.Commands.Outtake;
import frc.robot.Commands.Shooting;
import frc.robot.Commands.SpinUp;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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

        autoChooser = AutoBuilder.buildAutoChooser();

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