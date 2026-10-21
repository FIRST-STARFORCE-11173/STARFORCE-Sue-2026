package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Targets;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAim extends Command {

    private final DriveSubsystem drive;

    public AutoAim(
            DriveSubsystem drive) {

        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {

        Pose2d target =
            DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red

                    ? Targets.RED_TARGET
                    : Targets.BLUE_TARGET;

        double rotation =
            drive.getAimRotation(target);

        drive.drive(
            0,
            0,
            MathUtil.clamp(
                rotation,
                -0.7,
                0.7));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}