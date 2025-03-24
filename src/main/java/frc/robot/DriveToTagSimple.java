package frc.robot;
// SIMPLE CODE: Drive toward AprilTag using Limelight only (no field layout)
// Aligns and drives to tag without knowing field position
// 0.8 to 1.2 meters (~2.5 to 4 feet) away


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.LimelightVision;

public class DriveToTagSimple extends Command {
    private final SwerveSubsystem swerve;
    private final LimelightVision vision;

    private final PIDController strafePID = new PIDController(0.03, 0, 0.002);
    private final PIDController rotationPID = new PIDController(0.015, 0, 0.001);
    private final double stopDistanceMeters;
    private double startTime;

    public DriveToTagSimple(SwerveSubsystem swerve, LimelightVision vision, double stopDistanceMeters) {
        this.swerve = swerve;
        this.vision = vision;
        this.stopDistanceMeters = stopDistanceMeters;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        System.out.println("DRIVETOSIMPLE INIT");
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

   System.out.println( NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));

        if (!vision.hasTarget()) {
            swerve.drive(new Translation2d(0, 0), 0, false);
            return;
        }

        double tx = vision.getTX(); // Horizontal offset (deg)
        double ty = vision.getTY(); // Vertical offset (deg)
        double z = vision.getCameraToTagZ(); // Forward distance to tag (m)

        // Calculate strafe (side-to-side) and rotation commands
        double strafe = strafePID.calculate(tx, 0); // Want to center tag horizontally
        double rotate = rotationPID.calculate(tx, 0); // Also rotate to face the tag

        System.out.println("TX , TY , Z , strafe , rotate : " + tx + " , " + ty + " , " + z + " , " + strafe + " , " + rotate);

        double forward = 0.5; // Constant forward speed
        if (z < stopDistanceMeters + 0.1) {
            forward = 0; // Stop when close
            System.out.println("DONE REACHED TAG");
    
        }

        swerve.drive(new Translation2d(forward, strafe), rotate, true);
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() && vision.getCameraToTagZ() < stopDistanceMeters;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false);
    }
} 

