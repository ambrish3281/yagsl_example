// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.spark.SparkBase;
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

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  // Setup the variables
  private SparkMax motor_intake_one;
  private SparkMax motor_intake_two;
  private SparkMax motor_intake_three;
  private SparkMax motor_elevator_one;
  private SparkMax motor_elevator_two;
  private XboxController operator_controller;
  private Joystick new_joystick;
  private DigitalInput intakesensor;
  private SparkMax motor_arm;
  private SparkMax motor_algae;

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
    // Setup CAN ID 9 for the Intake Motor Test
    //AM Comment test. We need to update deviceId from 13 onwards
    //motor_elevator_one = new SparkMax(21, MotorType.kBrushless); // SparkMax is flashed to CAN id 9
    //motor_elevator_two = new SparkMax(22, MotorType.kBrushless); // SparkMax is flashed to CAN id
    motor_intake_one = new SparkMax(23, MotorType.kBrushless);
    motor_arm = new SparkMax(25, MotorType.kBrushless);
    motor_algae = new SparkMax(24, MotorType.kBrushless);
    intakesensor = new DigitalInput(1);

    SparkMaxConfig config_ = new SparkMaxConfig();
    //motor_intake_one.getEncoder().setPosition(0);

    
    config_.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config_.smartCurrentLimit(80);
    config_.openLoopRampRate(0.2);
    //config_.idleMode(IdleMode.kBrake);
    
    motor_intake_one.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    motor_algae.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    motor_arm.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    
    // Operator Controller Port
    operator_controller = new XboxController(1);

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
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    // Get the controller axis and button values ONCE and save to vars
    double rawaxis1 = operator_controller.getRawAxis(1);
    double rawaxis2 = operator_controller.getRawAxis(2);
    double rawaxis3 = operator_controller.getRawAxis(3);
    double rawaxis5 = operator_controller.getRawAxis(5);
    boolean leftbumperbutton = operator_controller.getLeftBumperButton();
    boolean rightbumperbutton = operator_controller.getRightBumperButton();

    System.out.println("JOystick rawxis1 : " + rawaxis1/2 +
     " : Sensor output .get() : " + intakesensor.get() + 
     " : left bumper : " + leftbumperbutton + 
     " : right bumper : " + rightbumperbutton);

    // CORAL INTAKE
    if(intakesensor.get() == true)
    {
      motor_intake_one.set(rawaxis2/5);
    }else
    {
      motor_intake_one.set(0);
    }

    // CORAL SHOOT
    if(rawaxis3 > 0)
    {
      motor_intake_one.set(rawaxis3/5);
    }

    // ARM
      motor_arm.set(rawaxis5); 
    
    // ALGAE    
      if(leftbumperbutton)
      {
        motor_algae.set(0.2);
      } else if (rightbumperbutton)
      {
        motor_algae.set(-0.2);
      } else {
        motor_algae.set(0);
      }
        
    // ELEVATOR
    // motor_elevator_two.set(rawaxis1);
    // motor_elevator_one.set(-rawaxis1);

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
