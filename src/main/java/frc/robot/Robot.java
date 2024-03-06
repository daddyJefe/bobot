// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  final AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drive = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);


  @Override 
  public void robotInit(){
    
  }
  @Override
  public void autonomousPeriodic() {
    m_drive.updateOdometry();
    float roboDirection = ahrs.getYaw();

  roboDirection += 180;

  

    m_drive.getDirection(2, 0);
    
      /*
       *if(roboDirection > m_drive.getDirection(0,0)){
      System.out.println("turn right");
      m_drive.doArcadeDrive(0, -.3);
    }else{
      m_drive.doArcadeDrive(0, .3);
      System.out.println("print left");
    } 
       * 
       * 
       * 
       * 
       * 
       * 
       * 
       */
      //System.out.println((m_drive.getDirection(2, 0)-roboDirection)*.1/100);
      
    if(degreesToTurn(ahrs.getYaw(), m_drive.getDirection(2,0)) > 2){
      m_drive.doArcadeDrive(0, .3);
   
    }else if(degreesToTurn(ahrs.getYaw(), m_drive.getDirection(2,0)) < -2){
      m_drive.doArcadeDrive(0, -.3);
    }else{
      //m_drive.doArcadeDrive(0, degreesToTurn(ahrs.getYaw(), m_drive.getDirection(2,0))/360);
      if(m_drive.pointRange(2, 0)){
       m_drive.doArcadeDrive(-.3, 0);
      }
    }


    m_drive.pointRange(2,0);
  }

  public double degreesToTurn(double x, double y) {
    // Calculate the difference between the two directions
    x = x + 180;
    double turn = y - x;

    // Normalize the turn to be between -180 and 180 degrees

    System.out.println(turn);
    if(turn< 10 && turn > -10){
      turn = 0;
    }
    return turn;
    
}

  @Override
  public void teleopPeriodic() {
    
    m_drive.doArcadeDrive(m_controller.getLeftY(), m_controller.getRightX());
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_speedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;
    
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;

   

    

   System.out.println(m_drive.getDirection(.5, .5));
  }

  public void robotPeriodic(){



    
  }
}