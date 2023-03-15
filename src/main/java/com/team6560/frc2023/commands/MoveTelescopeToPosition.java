// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveTelescopeToPosition extends CommandBase {
  Telescope m_telescope;
  int position;
  /** Creates a new MoveTelescopeToPosition. */
  public MoveTelescopeToPosition(Telescope t, int p) {
    m_telescope = t;
    position = p;
    addRequirements(t);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_telescope.MoveTelescopeToPosition(position);
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
