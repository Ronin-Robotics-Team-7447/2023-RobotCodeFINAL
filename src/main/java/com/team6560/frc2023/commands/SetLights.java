// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLights extends CommandBase {
  Lights m_lights;
  String mode;
  /** Creates a new SetLights. */
  public SetLights(Lights l, String m) {
    m_lights = l;
    mode = m;
    addRequirements(l);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mode == "cone") {
      m_lights.setLightsToCone();
    } else if(mode == "cube") {
      m_lights.setLightsToCube();
    } else if(mode == "default") {
      m_lights.setLightsToDefault();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
