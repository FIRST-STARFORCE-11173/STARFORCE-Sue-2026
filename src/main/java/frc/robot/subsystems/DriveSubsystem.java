package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final AHRS m_gyro =
        new AHRS(NavXComType.kMXP_SPI);

    private final MAXSwerveModule m_frontLeft =
        new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight =
        new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft =
        new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight =
        new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private boolean m_fieldOriented = true;

    public DriveSubsystem() {

        m_gyro.reset();

        m_poseEstimator =
            new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(getGyroAngle()),
                getModulePositions(),
                new Pose2d());

        configureAutoBuilder();
    }

    @Override
    public void periodic() {

        m_poseEstimator.update(
            Rotation2d.fromDegrees(getGyroAngle()),
            getModulePositions());
    }

    public void drive(
            double xSpeed,
            double ySpeed,
            double rot) {

        double xSpeedDelivered =
            xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;

        double ySpeedDelivered =
            ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

        double rotDelivered =
            rot * DriveConstants.kMaxAngularSpeed;

        ChassisSpeeds speeds;

        if (m_fieldOriented) {

            speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(getGyroAngle()));

        } else {

            speeds =
                new ChassisSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered);
        }

        driveRobotRelative(speeds);
    }

    public void driveRobotRelative(
            ChassisSpeeds chassisSpeeds) {

        SwerveModuleState[] states =
            DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_rearLeft.setDesiredState(states[2]);
        m_rearRight.setDesiredState(states[3]);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {

        m_poseEstimator.resetPosition(
            Rotation2d.fromDegrees(getGyroAngle()),
            getModulePositions(),
            pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {

        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState());
    }

    private void configureAutoBuilder() {

        try {

            RobotConfig config =
                RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,

                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
                ),

                config,

                () -> DriverStation.getAlliance()
                    .map(
                        alliance ->
                        alliance == DriverStation.Alliance.Red)
                    .orElse(false),

                this);

        } catch (Exception e) {

            DriverStation.reportError(
                "Failed to configure AutoBuilder",
                e.getStackTrace());
        }
    }

    public double getGyroAngle() {
        return -m_gyro.getYaw();
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public void toggleFieldOriented() {
        m_fieldOriented = !m_fieldOriented;
    }

    public boolean isFieldOriented() {
        return m_fieldOriented;
    }

    public SwerveModulePosition[] getModulePositions() {

        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
    }

    public void setModuleStates(
            SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {

        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearLeft.resetEncoders();
        m_rearRight.resetEncoders();
    }

    public void setX() {

        m_frontLeft.setDesiredState(
            new SwerveModuleState(
                0,
                Rotation2d.fromDegrees(45)));

        m_frontRight.setDesiredState(
            new SwerveModuleState(
                0,
                Rotation2d.fromDegrees(-45)));

        m_rearLeft.setDesiredState(
            new SwerveModuleState(
                0,
                Rotation2d.fromDegrees(-45)));

        m_rearRight.setDesiredState(
            new SwerveModuleState(
                0,
                Rotation2d.fromDegrees(45)));
    }
}