package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;




public class AutoCoralRaise extends Command {

    private double inchdistprivate = 0;
    
    public AutoCoralRaise(double inchesdist) {

        inchdistprivate = inchesdist;
    
    }


    @Override
    public void initialize() {

            Robot.targetRotations = inchdistprivate * 1.82;
        //motor_elevator_one_encoder.setPosition(0); // Reset encoder before starting
        Robot.movingToTarget = true;
        System.out.println( " : INIT targetRotations : " + Robot.targetRotations);
        System.out.println( " : INIT CURRENT POSITION : " + Robot.motor_elevator_one_encoder.getPosition());
        System.out.println( " : INIT ERRORVAL  : " + (Robot.targetRotations - Robot.motor_elevator_one_encoder.getPosition()));

    }

    @Override
    public void execute() {


        
        if (Robot.movingToTarget)
        {
            double currentPosition = Robot.motor_elevator_one_encoder.getPosition();
            double error = Robot.targetRotations - currentPosition;

            /* 
            Robot.motor_elevator_one.set(0.25 * Math.signum(error)); // Apply PID output
            Robot.motor_elevator_two.set(0.25 * Math.signum(error)); // Apply PID output
*/

if(Math.signum(error) > 0) // UP SPEED SET
    {
      Robot.motor_elevator_two.set(0.25); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      Robot.motor_elevator_one.set(0.25); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      
      //System.out.println("ELE SPEED GOIN UP: " + motor_elevator_one.get());

    }else                       // DOWN SPEED SET
    {
      Robot.motor_elevator_two.set(-0.15); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      Robot.motor_elevator_one.set(-0.15); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      //System.out.println("ELE SPEED GOIN DOWN : " + motor_elevator_one.get());

    }
            
    /* *
            System.out.println("IN AUTO CORAL RAISE : currentPosition : " + currentPosition
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
                    System.out.println("IN CORAL END");

    }
}

