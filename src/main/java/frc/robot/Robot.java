// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

// AUTO TEST
    private Pose2d initialPose;
    private boolean ranAutoTest = false;
    private Timer timer = new Timer();


  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  // Setup the variables
  public static SparkMax motor_intake_one;
  public static SparkMax motor_intake_two;
  public static SparkMax motor_intake_three;
  public static SparkMax motor_elevator_one;
  public static SparkMax motor_elevator_two;

  // STUFF FOR ELEVATOR PID CONTROLLER FIXED POSITIONS
  public static RelativeEncoder motor_elevator_one_encoder;
  public static RelativeEncoder motor_elevator_two_encoder;
  public static PIDController motor_elevator_one_pidc = new PIDController(0.2, 0, 0.001); // Tune PID values
  public static PIDController motor_elevator_two_pidc = new PIDController(0.2, 0, 0.001); // Tune PID values
  public static double WHEEL_CIRCUMFERENCE = 0.159593026;
  public static double GEAR_RATIO = 40;
  public static double INIT_ENCODER ;
  public static double targetRotations = 0;
  public static boolean movingToTarget = false;

  // ALGAE

  public static SparkFlex algae_raise_motor;
  public static SparkFlex algae_spin_motor;
  public static SparkClosedLoopController  algae_raise_motor_pidc; // Tune PID values
  public static RelativeEncoder  algae_raise_motor_encoder;

            
  private XboxController operator_controller;
  private Joystick new_joystick;
  private DigitalInput intakesensor;
  private SparkFlex motor_arm;
  private SparkMax motor_algae;
  public static int alliancelocation = 999;
  public static String robotalliance = "NONE";
  private boolean wasTeleop = false;


  public static LimelightVision lv;

  /* We can add     enableLiveWindowInTest(true);
 to see live data in Smartboard Ambrish */
  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
  

    // ALLIANCE AND LOC - DEFINSIVE LOGIC DUE TO SOME WHACKY WPILIB LOGIC

    if(DriverStation.getAlliance().isPresent() == true)
    {
      Alliance ra = DriverStation.getAlliance().get();
      robotalliance = ra.toString();
    }

    if(DriverStation.getLocation().isPresent() == true)
    {
      alliancelocation = DriverStation.getLocation().getAsInt();
    }

    System.out.println("DS ALLIANCE : " + robotalliance);
    System.out.println("DS ALLIANCE LOCATION : " + alliancelocation);


    // ************************  FOR TEST PLS REMOVE 
    //robotalliance = "Red";
    //alliancelocation = 999;
    // ************************  FOR TEST PLS REMOVE 


    System.out.println("final ALLIANCE : " + robotalliance);
    System.out.println("final ALLIANCE LOCATION : " + alliancelocation);

    intakesensor = new DigitalInput(1);

    motor_intake_one = new SparkMax(23, MotorType.kBrushless);
     //motor_arm = new SparkFlex(31, MotorType.kBrushless);
    //motor_algae = new SparkMax(24, MotorType.kBrushless);
    motor_elevator_one = new SparkMax(21, MotorType.kBrushless); // SparkMax is flashed to CAN id 9
    motor_elevator_two = new SparkMax(22, MotorType.kBrushless); // SparkMax is flashed to CAN id
    SparkMaxConfig config_ = new SparkMaxConfig();        
    config_.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motor_intake_one.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    motor_elevator_one.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    motor_elevator_two.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //motor_algae.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //motor_arm.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    motor_elevator_one_encoder = motor_elevator_one.getEncoder();
    motor_elevator_two_encoder = motor_elevator_two.getEncoder();
    System.out.println("ELE1 POSITION BEFORE SET 0:" + motor_elevator_one_encoder.getPosition());  
    System.out.println("ELE2 POSITION BEFORE SET 0:" + motor_elevator_two_encoder.getPosition());  
    System.out.flush();
    System.out.flush();
    motor_elevator_one_encoder.setPosition(0);
    motor_elevator_two_encoder.setPosition(0);  
    Timer.delay(0.02);
    System.out.println("ELE1 POSITION AFTER SET 0:" + motor_elevator_one_encoder.getPosition());  
    System.out.println("ELE2 POSITION AFTER SET 0:" + motor_elevator_two_encoder.getPosition());  
    System.out.flush();
    System.out.flush();
    INIT_ENCODER = motor_elevator_one_encoder.getPosition(); // INITIAL ENCOER VALUE
    movingToTarget = false;
    // Operator Controller Port
    operator_controller = new XboxController(1);



    // APRIL TAG SETUP
    lv = new LimelightVision();
    
    

    // ALGAE SETUP

    algae_spin_motor = new SparkFlex(32, MotorType.kBrushless); // SparkMax is flashed to CAN id
    
    
    algae_raise_motor = new SparkFlex(31, MotorType.kBrushless); // SparkMax is flashed to CAN id
    SparkFlexConfig config2_ = new SparkFlexConfig();        
    config2_.idleMode(SparkBaseConfig.IdleMode.kBrake);
    ClosedLoopConfig pidConfig2 = new ClosedLoopConfig();
    pidConfig2.pid(0.5,0,0);
    config2_.apply(pidConfig2);
    algae_raise_motor.configure(config2_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    algae_raise_motor_encoder = algae_raise_motor.getEncoder();
    algae_raise_motor_encoder.setPosition(0.0); // Assume arm is hanging down at boot

    // Set up onboard PID constants
    algae_raise_motor_pidc = algae_raise_motor.getClosedLoopController();

    Timer timeralg = new Timer();
    timeralg.reset();
    timeralg.start();
    

    System.out.println("RAISING ARM START <<<<<<<<<<<<<<<<<<<<<<<");
    // Run the motor for 2 seconds
    while (timeralg.get() < 2.0) {
     // System.out.println("RAISING ARM RUNNING <<<<<<<<<<<<<<<<<<<<<<<");

        algae_raise_motor.set(0.15);
    }
    
    // Optionally stop the motor after 2 seconds
    algae_raise_motor.set(0.04); // feed forward    

    System.out.println("RAISING ARM END <<<<<<<<<<<<<<<<<<<<<<<<<");
    // Left Hand Joystick Port
    // new_joystick = new Joystick(1);

    // XboxController exampleController = new XboxController(0); // Creates an XboxController on port 2.
    // Trigger yButton = new JoystickButton(exampleController, XboxController.Button.kLeftBumper.value); // Creates a new JoystickButton object for the `Y` button on exampleController
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();


    // remove ELE brake mode
    /* 
    if (wasTeleop) {

      SparkMaxConfig config_ = new SparkMaxConfig();        
      config_.idleMode(SparkBaseConfig.IdleMode.kCoast);
      motor_elevator_one.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      motor_elevator_two.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
      System.out.println("TELEOP DISABLED : ELE MOTOR NOW IN COAST MODE , NOT BRAKE ");
      wasTeleop = false; // Reset the flag
    }
      */
      
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {



    
    // ALLIANCE AND LOC - DEFINSIVE LOGIC DUE TO SOME WHACKY WPILIB LOGIC

    if(DriverStation.getAlliance().isPresent() == true)
    {
      Alliance ra = DriverStation.getAlliance().get();
      robotalliance = ra.toString();
    }

    if(DriverStation.getLocation().isPresent() == true)
    {
      alliancelocation = DriverStation.getLocation().getAsInt();
    }

    System.out.println("DS ALLIANCE : " + robotalliance);
    System.out.println("DS ALLIANCE LOCATION : " + alliancelocation);


    // ************************  FOR TEST PLS REMOVE 
    //robotalliance = "Red";
    //alliancelocation = 999;
    // ************************  FOR TEST PLS REMOVE 


    System.out.println("final ALLIANCE : " + robotalliance);
    System.out.println("final ALLIANCE LOCATION : " + alliancelocation);



    // Ensure sensors are zeroed (like gyro, if needed)
    
/* 
    // Force odometry to update once — initializes m_poseMeters
    SwerveDrive sd = m_robotContainer.drivebase.getSwerveDrive(); 
    sd.zeroGyro();
    sd.updateOdometry(); 
    // Now it's safe to reset odometry to your desired starting pose
    sd.resetOdometry(new Pose2d(
        new Translation2d(0, 0),
        Rotation2d.fromDegrees(0)
    ));
*/


    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println(">>>>>>>>>>>>>>>>>> IN AUTO :" + m_autonomousCommand.getName());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }


    // AUTO TEST
    /*
    initialPose = m_robotContainer   .getPose();
    timer.reset();
    timer.start();
    ranAutoTest = false;
    System.out.println("Starting test: initial pose = " + initialPose.getTranslation());
    */
  }
    
    
  

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {

    
    // motor_elevator_one.set(0.1);
    //motor_intake_one.set(controller.getY(Hand.kLeft));
    // motor_intake_one.set(controller.getY(XboxController.Button.kA));
  
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, rehmove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
    /*Below line can be commented out  based on feedback */
    m_robotContainer.setDriveMode();
    System.out.println(movingToTarget);
    
    movingToTarget = false;
    System.out.println("movingToTarget : " + movingToTarget);

    wasTeleop = true;
    System.out.println("wasTeleop : " + wasTeleop);

  }



  
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    // Get the controller axis and button values ONCE and save to vars
    double rawaxis1 = operator_controller.getRawAxis(1);
    ///////////////////
    double rawaxis2 = operator_controller.getRawAxis(2);
    double rawaxis3 = operator_controller.getRawAxis(3);
    double rawaxis5 = operator_controller.getRawAxis(5);
    boolean leftbumperbutton = operator_controller.getLeftBumperButton();
    boolean rightbumperbutton = operator_controller.getRightBumperButton();
    int opPOV = operator_controller.getPOV();

    /* 
    System.out.println("rawxis1 : " + rawaxis1 +
     " : Sensor.get() : " + intakesensor.get() + 
     " : ELE1 POS : " + motor_elevator_one_encoder.getPosition() +
     " : ELE2 POS : " + motor_elevator_two_encoder.getPosition());
    
     
     System.out.println("x  : " + operator_controller.getXButton() +
     " : " + operator_controller.getAButton() +
     " : " + operator_controller.getYButton() +
     " : " + operator_controller.getBButton() );

     System.out.flush();
     System.out.flush();

*/


    // ALGAE

  //  System.out.println("ALGAE SPIN ROTATIONS : " + algae_spin_motor.getEncoder().getPosition());
   // System.out.println("ALGAE ARM ROTATIONS : " + algae_raise_motor.getEncoder().getPosition());

   // System.out.println("opPOV : " + opPOV);


    if (leftbumperbutton)
    {
      //System.out.println("PRESSED LEFT bumper ");
      algae_spin_motor.set(0.2);
    }else if (rightbumperbutton)
    {
      //System.out.println("PRESSED RIGHT bumper ");
      algae_spin_motor.set(-0.2);
    }else 
    {
      algae_spin_motor.set(0);
    }


    if (rawaxis5 > 0.05)
    {
      //System.out.println("PRESSED RIGHT JOYST POSITIVE ");
      algae_raise_motor.set(-0.02);
    }else if (rawaxis5 < -0.05) // UP
    {
      //System.out.println("PRESSED RIGHT JOYST NEGETIVE ");
      algae_raise_motor.set(0.15);
    }else 
    {
      //System.out.println("ALGAE ARM STOP ");
      algae_raise_motor.set(0.04); // MIGHT NEED FEED FORWARD
    }



    /* 
    if (opPOV == 0) {
         algae_raise_motor_pidc.setReference(3.0, ControlType.kPosition, ClosedLoopSlot.kSlot0); // Use PID slot 0
        //algae_raise_motor_pidc.setReference(3,ControlType.kPosition);   
    }
    */


        // CORAL INTAKE
    /////////////////
    if(intakesensor.get() == true)
    {
      if(rawaxis2 > 0)
      {
      motor_intake_one.set(0.2);
      }else
      {
              motor_intake_one.set(0);

      }
    }else
    {
      motor_intake_one.set(0);
    } 

    // CORAL SHOOT
    if(rawaxis3 > 0)
    {
      motor_intake_one.set(0.22);
      //motor_intake_one.set(rawaxis3/4);
         // System.out.println("shoot speed : " + 0.1);

    } 

    /* 
    if (rightbumperbutton)
    {
            motor_intake_one.set(0.11111111111);
            System.out.println("Rightbumper shoot speed : " + 0.11111111111);


    }
            */

    // ARM
      //motor_arm.set(rawaxis5); 
    
    // ALGAE    
    //  if(leftbumperbutton)
      //{
        //motor_algae.set(0.2);
     // } else if (rightbumperbutton)
      //{
        //motor_algae.set(-0.2);
      //} else {
        //motor_algae.set(0);
      //}
        
    // ELEVATOR

//          System.out.println("ELE SPEED IN PERIODIC: " + motor_elevator_one.get());

    /* 
    if (Math.abs(rawaxis1) < 0.05)
    {
      System.out.println("JOYSTICK ELE MOVE - ENC CURR POS : " + motor_elevator_one_encoder.getPosition() );
      double spd = 0;
      if(motor_elevator_one.getEncoder().getPosition() < 0.08) 
      {
        spd = 0; // TURN OFF MOTOR IF CLOSE TO HOME
      }else{
        spd = 0.05; // FEED FORWARD
      }
      motor_elevator_two.set(spd);
      motor_elevator_one.set(spd);
      System.out.println("ELE SPEED AT 0 JOYST: " + motor_elevator_two.get());

    }else if (rawaxis1 < 0.05) // UP
    {
      motor_elevator_two.set(0.4); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      motor_elevator_one.set(0.4);
    //System.out.println("ELE UP : " + 0.3);
   
    } else if (rawaxis1 > 0.05)                 // DOWN
    {
    motor_elevator_two.set(0.03); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
    motor_elevator_one.set(0.03);
    //System.out.println("ELE DOWN : " + 0.03);

    } // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN

    */

    
    // TEST TEST TEST
    
   if (operator_controller.getXButton() == true) { // HOME = X BUTTON
     moveDistance(0.0); // 
   }


   //L! MAGUC NUMBER IS 25.25 INCH
   if (operator_controller.getYButton() == true) { // L1 = Y BUTTON
    moveDistance(6); // Move 1 meter
  }

  if (operator_controller.getBButton() == true) { // L2 = B BUTTON
    moveDistance(15); // Move 1 meter
  }

  if (operator_controller.getAButton() == true) { // L3 / TESTING L1 FOR NOW ***********************
    moveDistance(30); // Move 1 meter
  }

  // Run PID control if we are actively moving to a target
  
  if (movingToTarget)
  {
    double currentPosition = motor_elevator_one_encoder.getPosition();
    double error = targetRotations - currentPosition;
    double rawpidoutput = motor_elevator_one_pidc.calculate(currentPosition, targetRotations) ;
    //double pidOutput =  MathUtil.clamp(rawpidoutput,-0.15,0.25);
    double pidOutput = scale(rawpidoutput,-1,1,-0.15,0.25);
    //System.out.println("RAW PID : " + rawpidoutput + " : CLAMPED : " + pidOutput);
    //motor_elevator_two.set(pidOutput);
    //motor_elevator_one.set(pidOutput);


    
    if(Math.signum(error) > 0) // UP SPEED SET
    {
      motor_elevator_two.set(0.25); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      motor_elevator_one.set(0.25); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      
      //System.out.println("ELE SPEED GOIN UP: " + motor_elevator_one.get());

    }else                       // DOWN SPEED SET
    {
      motor_elevator_two.set(-0.2); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      motor_elevator_one.set(-0.2); // NO NEGETIVE NEEDED FOR ONE MOTOR DUE TO DESIGN
      //System.out.println("ELE SPEED GOIN DOWN : " + motor_elevator_one.get());

    }
      
      

    //motor_elevator_one.set(0.3 * Math.signum(error)); // Apply PID output
    //motor_elevator_two.set(0.3 * Math.signum(error)); // Apply PID output


    SmartDashboard.putNumber("Motor Position", currentPosition);
    SmartDashboard.putNumber("Target Position", targetRotations);
    SmartDashboard.putNumber("Motor Power", pidOutput);
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("TEST VALUE", 55);


    /* 
    System.out.println("currentPosition : " + currentPosition
    + " : targetRotations : " + targetRotations
    + " : pidOutput : " + pidOutput
    + ": error :" + error);
    */
    


    // Stop if close enough OR if the PID output changes direction (prevents overshoot)
    if ((Math.abs(error) < 0.1) && targetRotations == 0) 
    {
      System.out.println("AT HOME HEIGHT - SETTING 0 FEED FORWARD");
      System.out.println( " : errorval : " + error + ": currentPosition : " + motor_elevator_one_encoder.getPosition()) ;

        motor_elevator_two.set(0.0);
        motor_elevator_one.set(0.0);
        //System.out.println("ELE SPEED : " + motor_elevator_one.get());


        movingToTarget = false;
    } else if((Math.abs(error) < 0.1)){
      System.out.println("REACHED : " + targetRotations);
      System.out.println( " : errorval : " + error + ": currentPosition : " + motor_elevator_one_encoder.getPosition()) ;

        motor_elevator_two.set(0.02); // FEED FORWARD
        motor_elevator_one.set(0.02);
        //System.out.println("ELE SPEED : " + motor_elevator_one.get());

        movingToTarget = false;


    }
  }}
        

  public void moveDistance(double inchesdist) {
    //targetRotations = (meters / WHEEL_CIRCUMFERENCE) * GEAR_RATIO;
    //targetRotations = inchesdist * 0.395;
    targetRotations = inchesdist * 1.82;
    //motor_elevator_one_encoder.setPosition(0); // Reset encoder before starting
    motor_elevator_one_pidc.reset();
    movingToTarget = true;
    System.out.println( " : targetRotations : " + targetRotations);
    System.out.println( " : CURRENT POSITION : " + motor_elevator_one_encoder.getPosition());
    System.out.println( " : ERRORVAL  : " + (targetRotations - motor_elevator_one_encoder.getPosition()));
}

  public double scale(double value, double inMin, double inMax, double outMin, double outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
  }
    
    



  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setDriveMode();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
