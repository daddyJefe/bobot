// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  private final CANSparkMax m_leftLeader = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax m_leftFollower = new CANSparkMax(2, MotorType.kBrushless);
      private final CANSparkMax m_rightLeader = new CANSparkMax(4, MotorType.kBrushless);
        private final CANSparkMax m_rightFollower = new CANSparkMax(3, MotorType.kBrushless);
  
  DifferentialDrive bobotDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  

  RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();
  //AHRS ahrs = new AHRS(SPI.Port.kMXP);
  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  //AnalogGyro AnalogGyro = ahrs.getRotation2d();
  
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    ahrs.zeroYaw();

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
 
    m_odometry =
        new DifferentialDriveOdometry(
            ahrs.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), 
            new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
    //m_leftLeader.set(leftOutput + leftFeedforward);
    //m_rightLeader.set(rightOutput + rightFeedforward);


  }

  public void doArcadeDrive(double controllerX, double controllerY){

        bobotDrive.arcadeDrive(controllerX , controllerY);

  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  
  
  public double getDirection(double x,double y) {
    // Calculate the angle between the two points
    Pose2d odo2 = m_odometry.getPoseMeters();
    updateOdometry();
    odo2 = m_odometry.getPoseMeters();
    double angle = Math.atan2(odo2.getY() - y,-1*odo2.getX() - x) * (180 / Math.PI);

    // Normalize the angle to be between 0 and 360 degrees
    if (angle < 0) {
        angle += 360;
    }
    //return odo2.getY();
    return angle;
}

public boolean pointRange(double x, double y){
   updateOdometry();
   Pose2d odo3 = m_odometry.getPoseMeters();
    odo3 = m_odometry.getPoseMeters();
  if(Math.abs(odo3.getY()-y)<1 && Math.abs(odo3.getX()-x)<1){
    
    return false;
  }
  //System.out.print(Math.abs(odo3.getY()-y));
  System.out.println(Math.abs(odo3.getX()-x));
  return true;
  
}



  

}