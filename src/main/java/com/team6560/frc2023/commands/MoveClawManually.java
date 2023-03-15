// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveClawManually extends CommandBase {
  Claw m_claw;
  double clawSpeed;

  /** Creates a new MoveClawManually. */
  public MoveClawManually(Claw c, double s) {
    m_claw = c;
    clawSpeed = s;
    addRequirements(c);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("BUTTON WPRKING");
    m_claw.setClawSpeed(clawSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}