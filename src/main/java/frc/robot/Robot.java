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

 //private final XboxController m_controller = new XboxController(0);  
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
  //secondary differentialDrive created to drive the rear wheel systems
 private DifferentialDrive m_rearRobot;

  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;

private CANSparkMax m_leftRearMotor;
  private CANSparkMax m_rightRearMotor;


private final XboxController m_controller = new XboxController(0);  
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
    m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(4, MotorType.kBrushless);
     m_leftRearMotor = new CANSparkMax(2, MotorType.kBrushless);
    m_rightRearMotor = new CANSparkMax(3, MotorType.kBrushless);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
//m_leftMotor.setInverted(true);
    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_rearRobot = new DifferentialDrive(m_leftRearMotor, m_rightRearMotor);
    
  }

  @Override
  public void teleopPeriodic() {
   //m_myRobot.arcadeDrive(m_controller.getLeftY(), m_controller.getRightX());
  //m_rearRobot.arcadeDrive(m_controller.getLeftY(), m_controller.getRightX());
  }


  @Override
public void disabledPeriodic() {

}

@Override
public void robotPeriodic() {
  
}
}

