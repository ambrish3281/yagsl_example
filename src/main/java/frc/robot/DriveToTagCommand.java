package frc.robot;

// FINAL CODE: Drive to Specific AprilTag on Button Press
// Using Limelight 3 + WPILib AprilTag Layout + YAGSL 2025 Swerve


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.FieldConstants;
import frc.robot.LimelightVision;

public class DriveToTagCommand extends Command {
    private final SwerveSubsystem swerve;
    private final int tagId;
    private final double stopDistanceMeters;
    private Pose2d targetPose;
    private double startTime;

    private final LimelightVision vision;

    public DriveToTagCommand(SwerveSubsystem swerve, LimelightVision vision, int tagId, double stopDistanceMeters) {
        this.swerve = swerve;
        this.vision = vision;
        this.tagId = tagId;
        this.stopDistanceMeters = stopDistanceMeters;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();

        // Fuse odometry with Limelight pose if target is visible
        if (vision.hasTarget()) {
            Pose2d limelightPose = vision.getBotPose();
            double timestamp = Timer.getFPGATimestamp() - 0.03; // Limelight latency approx
            //swerve.addVisionMeasurement(limelightPose, timestamp);
            swerve.getSwerveDrive().addVisionMeasurement(limelightPose, timestamp);
        }

        Pose2d tagPose = FieldConstants.getTagPose2d(tagId);
        if (tagPose == null) {
            System.out.println("Invalid tag ID: " + tagId);
            cancel();
            return;
        }

        // Create target pose in front of tag
        Rotation2d tagRotation = tagPose.getRotation();
        Translation2d offset = new Translation2d(stopDistanceMeters, tagRotation);

        targetPose = new Pose2d(
            tagPose.getTranslation().plus(offset),
            tagRotation.plus(Rotation2d.fromDegrees(180)) // face the tag
        );

        swerve.driveToPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        double timeElapsed = Timer.getFPGATimestamp() - startTime;
        if (timeElapsed > 4.0) {
            System.out.println("Timeout: couldn't reach pose");
            return true;
        }

        if (targetPose == null) return true;

        Pose2d currentPose = swerve.getPose();
        double positionError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

        return positionError < 0.1 && angleError < 5;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        if (interrupted) {
            System.out.println("Drive to tag was interrupted.");
        } else {
            System.out.println("Reached target pose or timed out.");
        }
    }
}
