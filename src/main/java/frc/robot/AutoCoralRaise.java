package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;




public class AutoCoralRaise extends Command {

    
    public AutoCoralRaise(double inchesdist) {

        Robot.targetRotations = inchesdist * 1.82;
        //motor_elevator_one_encoder.setPosition(0); // Reset encoder before starting
        Robot.movingToTarget = true;
        System.out.println( " : targetRotations : " + Robot.targetRotations);
        System.out.println( " : CURRENT POSITION : " + Robot.motor_elevator_one_encoder.getPosition());
        System.out.println( " : ERRORVAL  : " + (Robot.targetRotations - Robot.motor_elevator_one_encoder.getPosition()));

    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {


        
        if (Robot.movingToTarget)
        {
            double currentPosition = Robot.motor_elevator_one_encoder.getPosition();
            double error = Robot.targetRotations - currentPosition;

            Robot.motor_elevator_one.set(0.25 * Math.signum(error)); // Apply PID output
            Robot.motor_elevator_two.set(0.25 * Math.signum(error)); // Apply PID output

            /* 
            System.out.println("IN CORAL RAISE : currentPosition : " + currentPosition
            + " : targetRotations : " + Robot.targetRotations
            + ": error :" + error);
            */


            // Stop if close enough OR if the PID output changes direction (prevents overshoot)
            if (Math.abs(error) < 0.1) 
            {
            System.out.println( " : errorval : " + error + ": currentPosition : " + Robot.motor_elevator_one_encoder.getPosition()) ;

            Robot.motor_elevator_one.set(0.02);
            Robot.motor_elevator_two.set(0.02);
            Robot.movingToTarget = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !Robot.movingToTarget;
    }

    @Override
    public void end(boolean interrupted) {
    }
}

