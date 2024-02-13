// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import java.nio.file.Path;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

 //private final XboxController m_controller = new XboxController(0);  
import com.revrobotics.CANSparkMax;

import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */



  /**
   * Imports a Trajectory from a JSON file exported from PathWeaver.
   *
   * @param path The path of the json file to import from
   * @return The trajectory represented by the file.
   * @throws IOException if reading from the file fails.
   */

 

public class Robot extends TimedRobot {
           
           AHRS ahrs = new AHRS(SPI.Port.kMXP);
       

   
  private DifferentialDrive m_myRobot;
  //secondary differentialDrive created to drive the rear wheel systems
  private DifferentialDrive m_rearRobot;
private Command autonomousCommand;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  int firing = 0;
  int loading = 0;
  private CANSparkMax m_leftRearMotor;
  private CANSparkMax m_rightRearMotor;

  //declaring motor encoders for data feedback!
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private CANSparkMax feedWheel;
  private CANSparkMax launchWheel;
  private final Timer m_Timer = new Timer();

private final XboxController m_controller = new XboxController(0);  

//Autonomous Mode Code
private static final String testAuto ="testAuto";
private static final String testAuto2 = "testAuto2";
private String m_AutoSelected;
private final SendableChooser<String> m_Chooser = new SendableChooser<>();


  @Override
  public void robotInit() {



    CameraServer.addServer("cam 1");
     CameraServer.addServer("cam 2");
    UsbCamera camera = CameraServer.startAutomaticCapture();
   
              // Set the resolution
              camera.setResolution(160, 120);

     UsbCamera camera2 = CameraServer.startAutomaticCapture();

              camera2.setResolution(160, 120);
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
    m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(4, MotorType.kBrushless);
     m_leftRearMotor = new CANSparkMax(2, MotorType.kBrushless);
    m_rightRearMotor = new CANSparkMax(3, MotorType.kBrushless);

    feedWheel = new CANSparkMax(5, MotorType.kBrushless);
    launchWheel = new CANSparkMax(6, MotorType.kBrushless);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
//m_leftMotor.setInverted(true);
    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_rearRobot = new DifferentialDrive(m_leftRearMotor, m_rightRearMotor);
    //this code sets up the encoders with their respective motors
    leftEncoder = m_leftRearMotor.getEncoder();
    rightEncoder = m_rightRearMotor.getEncoder();

    //Autonomous Mode Code
    m_Chooser.setDefaultOption("Default Test Auto", testAuto);
    m_Chooser.addOption("Option 2 Test", testAuto2);
    SmartDashboard.putData("Auto Choices", m_Chooser);
    
    
  }

  @Override
  public void teleopPeriodic() {
    
//do camera streaming
//CameraServer.startAutomaticCapture();





  m_myRobot.arcadeDrive(m_controller.getLeftY()*0.5,m_controller.getRightX()*0.25);
  m_rearRobot.arcadeDrive(m_controller.getLeftY()*0.5, m_controller.getRightX()*0.25);

  //this prints the positions of the motors in realtime
  SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
  SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
  SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
  SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
  SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
  SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
  SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

    if(m_controller.getRightTriggerAxis() > 0.1){

     
      System.out.println("X pressed");

      if(firing == 0){
        launchWheel.set(1);
        Timer.delay(.25);
         feedWheel.set(1);
         firing = 1;
         Timer.delay(.5);
         launchWheel.set(0);
         feedWheel.set(0);
         firing = 0;
     
    }
  }
    if(m_controller.getLeftTriggerAxis() > 0.1){

     
      System.out.println("A pressed");

      if(loading == 0){
        launchWheel.set(-.2);
         feedWheel.set(-.2);
         firing = 0;
         loading = 1;
        Timer.delay(.25);
     launchWheel.set(0);
     feedWheel.set(0);
        loading = 0;
      }
    }
  
    /** 
    if(m_Timer.get() < 2.0){
      launchWheel.set(0.9);
    }else if(m_Timer.get() < 4.0){
      launchWheel.set(0.9);
      feedWheel.set(0.9);
    }else{
      launchWheel.set(0.0);
      feedWheel.set(0.0);
    }
*/

  }


  @Override
public void disabledPeriodic() {

}

@Override
public void robotPeriodic() {
  
}

@Override
public void autonomousInit() {
m_AutoSelected = m_Chooser.getSelected();


}

@Override
public void autonomousPeriodic(){
  switch(m_AutoSelected){
    case testAuto2:
    //custom code goes here
    case testAuto:
    default:
    //Put default code here
    break;
  }


}

}

