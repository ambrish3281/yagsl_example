package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import edu.wpi.first.wpilibj.Timer;



public class AutoCoralShoot extends Command {

    private final Timer timer = new Timer();

    public AutoCoralShoot() {

    }


    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        Robot.motor_intake_one.set(0.28);
    }

    @Override
    public void execute() {
      // System.out.println("IN CORAL SHOOT");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(7);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.motor_intake_one.set(0);

    }
}

