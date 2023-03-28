// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmToAngle extends CommandBase {
  Arm m_arm;
  double angle;
  double speed;
  /** Creates a new MoveArmToAngle. */
  public MoveArmToAngle(Arm a, double inputAngle, double speed) {
    m_arm = a;
    angle = inputAngle;
    this.speed = speed;
    addRequirements(a);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmSpeedLimit(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( m_arm.getAbsoluteEncoderPosition() > angle ) {
      m_arm.setArmSpeedNoLimits(speed);
    } else if( m_arm.getAbsoluteEncoderPosition() < angle ) {
      m_arm.setArmSpeedNoLimits(-speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stopArm();
    m_arm.setArmSpeedLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( Math.abs(m_arm.getAbsoluteEncoderPosition() - angle ) < 3 ) {
      return true;
    } else {
      return false;
    }
  }
}
