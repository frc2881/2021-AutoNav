// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveWithJoysticks extends CommandBase {
private final Drive m_drive;
private final DoubleSupplier m_getDriverLeftyY;
private final DoubleSupplier m_getDriverRightX;


  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, DoubleSupplier getDriverLeftY, DoubleSupplier getDriverRightX) {
    m_drive = drive;
    m_getDriverLeftyY = getDriverLeftY;
    m_getDriverRightX = getDriverRightX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);

  }
//set up two double providers for left y and right x, pass double procider to drive w joysticks
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_getDriverLeftyY.getAsDouble(), m_getDriverRightX.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
