// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Wrist1;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveWristToAngle extends CommandBase {
  Wrist1 m_wrist;
  double angle;
  /** Creates a new MoveWristToAngle. */
  public MoveWristToAngle(Wrist1 w, double a) {
    m_wrist = w;
    angle = a;
    addRequirements(w);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setGoal(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
