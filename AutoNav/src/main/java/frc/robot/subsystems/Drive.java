// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DistancePerPulse;
import frc.robot.utils.NavX;

public class Drive extends SubsystemBase {
  
  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  private double distancePerPulse;
  private DifferentialDrive splitArcade;


  NavX m_gyro = new NavX(SPI.Port.kMXP);


  DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0);
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());


  public void drive(double left, double right)
  {
    splitArcade.arcadeDrive(left, right);
  }
  
  /** Creates a new Drive. */
  public Drive() { 


        leftFront = new CANSparkMax(1, MotorType.kBrushless);
        leftFront.setInverted(false);
        leftFront.setIdleMode(IdleMode.kBrake);

        leftRear = new CANSparkMax(2, MotorType.kBrushless);
        leftRear.setInverted(false);
        leftRear.setIdleMode(IdleMode.kBrake);

        // Set SlaveSpeedControllers to Follow MasterSpeedController
        leftRear.follow(leftFront);

        rightFront = new CANSparkMax(3, MotorType.kBrushless);
        rightFront.setInverted(false);
        rightFront.setIdleMode(IdleMode.kBrake);

        rightRear = new CANSparkMax(4, MotorType.kBrushless);
        rightRear.setInverted(false);
        rightRear.setIdleMode(IdleMode.kBrake);

        // Set SlaveSpeedControllers to Follow MasterSpeedController
        rightRear.follow(rightFront);    
        
        splitArcade = new DifferentialDrive(leftFront, rightFront);

        //(highestGearTeethNumber, lowestGearTeethNumber, wheelDiameter)
        distancePerPulse = DistancePerPulse.get(0, 0, 0);

        leftEncoder = leftFront.getEncoder();
        leftEncoder.setPositionConversionFactor(distancePerPulse);
        leftEncoder.setPosition(0);

        rightEncoder = rightFront.getEncoder();
        rightEncoder.setPositionConversionFactor(distancePerPulse);
        rightEncoder.setPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    m_odometry.update(m_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
      splitArcade.arcadeDrive(xSpeed, -zRotation, false);
    }

    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }

    public double getHeading(){
      return m_gyro.getRotation2d().getDegrees();
    }
  
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public void resetEncoders()
    {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
    }

    public void resetOdometry(Pose2d pose)
    {
      resetEncoders();
      m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        leftFront.setVoltage(leftVolts);
        leftRear.setVoltage(leftVolts);
        rightFront.setVoltage(-rightVolts);
        rightRear.setVoltage(-rightVolts);    
    }





}
