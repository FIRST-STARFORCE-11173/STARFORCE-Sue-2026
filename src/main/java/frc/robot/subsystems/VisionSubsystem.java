package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelight =
        NetworkTableInstance.getDefault()
            .getTable("limelight");

    public boolean hasTarget() {

        return limelight
            .getEntry("tv")
            .getDouble(0) == 1;
    }

    public double getTX() {

        return limelight
            .getEntry("tx")
            .getDouble(0);
    }

    public double[] getBotPoseBlue() {

        return limelight
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[6]);
    }
}