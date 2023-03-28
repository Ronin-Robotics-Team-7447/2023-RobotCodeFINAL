// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Wrist2;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveWristManually extends CommandBase {
  Wrist2 m_wrist;
  double wristSpeed;
  int buttonInput;

  /** Creates a new MoveWristManually. */
  public MoveWristManually(Wrist2 w, double s) {
    m_wrist = w;
    wristSpeed = s;
    addRequirements(w);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setWristSpeed(wristSpeed);
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