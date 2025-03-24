package frc.robot;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class LimelightVision extends SubsystemBase {
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTX() {
        return limelight.getEntry("tx").getDouble(0.0); // Horizontal offset in degrees
    }

    public double getTY() {
        return limelight.getEntry("ty").getDouble(0.0); // Vertical offset in degrees
    }

    public double getCameraToTagZ() {
        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        return pose.length >= 3 ? pose[2] : 99; // Z = forward distance
    }

public Pose2d getBotPose() {
    String entryName = "botpose_wpiblue"; // default
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
        entryName = "botpose_wpired";
    }

    double[] botpose = limelight.getEntry(entryName).getDoubleArray(new double[6]);
    if (botpose.length < 6) {
        return new Pose2d();
    }

    double x = botpose[0];
    double y = botpose[1];
    double yawDegrees = botpose[5];

    return new Pose2d(x, y, Rotation2d.fromDegrees(yawDegrees));
}

}
