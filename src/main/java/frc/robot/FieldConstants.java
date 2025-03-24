package frc.robot;


import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import java.util.Optional;

public class FieldConstants {

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2024Crescendo
    );

    public static Pose2d getTagPose2d(int tagId) {
        Optional<Pose3d> tagPose3d = fieldLayout.getTagPose(tagId);
        if (tagPose3d.isEmpty()) return null;

        Pose3d pose3d = tagPose3d.get();
        return new Pose2d(
            pose3d.getX(),
            pose3d.getY(),
            new Rotation2d(pose3d.getRotation().getZ())
        );
    }
}

