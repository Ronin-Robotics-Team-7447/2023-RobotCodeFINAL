// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Wrist1;
import com.team6560.frc2023.subsystems.Wrist2;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveWristToAngle extends CommandBase {
  Wrist2 m_wrist;
  double angle;
  double speed;
  /** Creates a new MoveWristToAngle. */
  public MoveWristToAngle(Wrist2 w, double a, double speed) {
    m_wrist = w;
    angle = a;
    this.speed = speed;
    addRequirements(w);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private void addRequirements(Wrist2 w) {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( m_wrist.getEncoderPosition() > angle ) {
      m_wrist.setWristSpeed(speed);
    } else if( m_wrist.getEncoderPosition() < angle ) {
      m_wrist.setWristSpeed(-speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( Math.abs(m_wrist.getEncoderPosition() - angle ) < 6 ) {
      return true;
    } else {
      return false;
    }
  }
}
