// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveClawManually extends CommandBase {
  Wrist m_wrist;
  double clawSpeed;

  /** Creates a new MoveClawManually. */
  public MoveClawManually(Wrist w, double s) {
    m_wrist = w;
    clawSpeed = s;
    addRequirements(w);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setClawSpeed(clawSpeed);
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
