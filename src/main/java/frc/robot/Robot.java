// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;

//yo
BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

private CANSparkMax m_leftMotor;
private CANSparkMax m_rightMotor;
private CANSparkMax m_leftMotor2;
private CANSparkMax m_rightMotor2;



private RelativeEncoder leftEncoder;
private RelativeEncoder rightEncoder;
//no
/** 
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(3);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(2);
  private final DifferentialDrive m_robotDrive =
    new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final PWMSparkMax m_leftDrive2 = new PWMSparkMax(1);
  private final PWMSparkMax m_rightDrive2 = new PWMSparkMax(4);
  private final DifferentialDrive m_robotDrive2 =
      new DifferentialDrive(m_leftDrive2::set, m_rightDrive2::set);
      */



  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

//stupid code goes here
double prevXAccel = 0.0;
double prevYAccel = 0.0;
// navX MXP using SPI
LinearFilter xAccelFilter = LinearFilter.movingAverage(10);
double filteredXAccel = 0.0;

//delete this if it is tweaking

  public Robot() {
   // SendableRegistry.addChild(m_robotDrive, m_leftDrive);
   // SendableRegistry.addChild(m_robotDrive, m_rightDrive);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
     /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
  m_leftMotor = new CANSparkMax(4, MotorType.kBrushless);
  m_rightMotor = new CANSparkMax(2, MotorType.kBrushless);

  /**
   * The RestoreFactoryDefaults method can be used to reset the configuration parameters
   * in the SPARK MAX to their factory default state. If no argument is passed, these
   * parameters will not persist between power cycles
   */
  m_leftMotor.restoreFactoryDefaults();
  m_rightMotor.restoreFactoryDefaults();

  m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    {
    }

  }
  

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
 
   // SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
//SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());


    //frontDrive.arcadeDrive(-m_controller.getLeftY(), m_controller.getRightX());
   // rearDrive.arcadeDrive(-m_controller.getLeftY(), m_controller.getRightX());
    
   m_myRobot.arcadeDrive(-1, 0);
   
  }

  @Override
  public void disabledPeriodic(){}

  @Override
  public void robotPeriodic(){}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  
}
