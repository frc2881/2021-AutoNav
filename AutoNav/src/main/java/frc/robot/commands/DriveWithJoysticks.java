// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class DriveWithJoysticks extends CommandBase {
private final Drive m_drive;
private final RobotContainer m_robotContainer;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, RobotContainer robotcontainer) {
    m_drive = drive;
    m_robotContainer = robotcontainer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = (m_robotContainer.getDriverLeftY());
    double zRotation = (m_robotContainer.getDriverRightX());
    m_drive.arcadeDrive(xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
