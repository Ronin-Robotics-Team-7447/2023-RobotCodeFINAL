// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.controls;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.Constants.*;
import com.team6560.frc2023.subsystems.Limelight;
import com.team6560.frc2023.subsystems.Telescope;
import com.team6560.frc2023.commands.DriveCommand;
import com.team6560.frc2023.utility.NumberStepper;
import com.team6560.frc2023.utility.PovNumberStepper;
import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ManualControls implements DriveCommand.Controls, Limelight.Controls {
  private XboxController xbox;

  private final PovNumberStepper speed;
  private final PovNumberStepper turnSpeed;

  private NetworkTable limelightTable;

  private NetworkTable climbTable;

  private NetworkTable intakeTable;

  private XboxController controlStation;

  private NetworkTable armTable;

  private boolean prevclimbEngaged;
  private boolean climbEngaged;

  Telescope m_telescope;
  /**
   * Creates a new `ManualControls` instance.
   *
   * @param xbox the Xbox controller to use for manual control
   */
  public ManualControls(XboxController xbox) {
    this(xbox, null);
  }

  public ManualControls(XboxController xbox, XboxController controlStation) {
    this.xbox = xbox;
    if (controlStation == null)
      this.controlStation = xbox;
    else
      this.controlStation = controlStation;

    this.speed = new PovNumberStepper(
        new NumberStepper(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.2 + (Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.15), 0.0,
            Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.6, Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.05),
        xbox,
        PovNumberStepper.PovDirection.VERTICAL);

    this.turnSpeed = new PovNumberStepper(
        new NumberStepper(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.1 + (Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.0025 * 5), 0.0,
            Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.15,
            Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.0025),
        xbox,
        PovNumberStepper.PovDirection.HORIZONTAL);
    
    ntDispTab("Controls")
      .add("Y Joystick", this::driveY)
      .add("X Joystick", this::driveX)
      .add("Rotation Joystick", this::driveRotationX);

    
    limelightTable = NetworkTableInstance.getDefault().getTable("Limelight");
    intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
    armTable = NetworkTableInstance.getDefault().getTable("Arm");
    intakeTable.getEntry("speed").setDouble(0.0);

    limelightTable.getEntry("limelightPipeline").setInteger( (long) 0);
    
    climbTable = NetworkTableInstance.getDefault().getTable("Climb");

    climbTable.getEntry("isClimbing").setBoolean(false);

    climbTable.getEntry("climbVelocityL").setDouble(0.0);

    climbTable.getEntry("climbVelocityR").setDouble(0.0);

    armTable.getEntry("resetArmZero").setBoolean(false);

    armTable.getEntry("overrideSoftLimits").setBoolean(false);

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2);

    // Square the axis
    value = Math.copySign(value * value, value);

    // Slew Rate Limiter
    // SlewRateLimiter limiter = new SlewRateLimiter(0.5);
    // value = limiter.calculate(value);
    
    return value;
  }

  /**
   * Returns the x component of the robot's velocity, as controlled by the Xbox
   * controller.
   *
   * @return the x component of the robot's velocity
   */
  @Override
  public double driveX() {
    /*if(m_telescope.getTelescopePosition() > Constants.TelescopeConstants.maxPositionForSlewToWork ) {
      return modifyAxis(xbox.getLeftY() * speed.get() * 0.5);
    }
    else return modifyAxis(xbox.getLeftY() * speed.get());*/
    return modifyAxis(-xbox.getLeftY() * speed.get());
  }

  /**
   * Returns the y component of the robot's velocity, as controlled by the Xbox
   * controller.
   *
   * @return the y component of the robot's velocity
   */
  @Override
  public double driveY() {
    // if(m_telescope.getTelescopePosition() > Constants.TelescopeConstants.maxPositionForSlewToWork ) {
    //   return modifyAxis(xbox.getLeftX() * speed.get() * 0.5);
    // }
    // else return modifyAxis(xbox.getLeftX() * speed.get());
    return modifyAxis(-xbox.getLeftX() * speed.get());
  }

  /**
   * Returns the angular velocity of the robot, as controlled by the Xbox
   * controller.
   *
   * @return the angular velocity of the robot
   */
  @Override
  public double driveRotationX() {
    return modifyAxis(-xbox.getRightX() * turnSpeed.get());
  }

  @Override
  public double driveRotationY() {
    return modifyAxis(-xbox.getRightY() * turnSpeed.get());
  }

  /**
   * Returns whether the yaw of the robot's gyroscope should be reset, as
   * controlled by the Xbox controller.
   *
   * @return whether the yaw of the robot's gyroscope should be reset
   */
  @Override
  public boolean driveResetYaw() {
    return xbox.getStartButton();
  }

  @Override
  public double driveBoostMultiplier() {
    if( xbox.getRightBumper() ) {
      return 0.5;
    } else if( xbox.getRightTriggerAxis() > 0.3 ) {
      return 1.5;
    } else return 1;
  }

  @Override
  public boolean autoAlign() {
    return xbox.getAButton();
  }

  @Override
  public boolean driveResetGlobalPose() {
    return xbox.getBackButton();
  }

  @Override
  public int getLimelightPipeline() {
    if (!DriverStation.isTeleop())
      return 0;
    else if (autoAlign()) {
      return 1;
    }
    return 1;
    // return (int) limelightTable.getEntry("limelightPipeline").getInteger( (long) 0);
  }

  @Override
  public boolean overrideMaxVisionPoseCorrection() {
    return xbox.getYButton();
  }

  @Override
  public boolean driveIsClimbing() {
    if (prevclimbEngaged != controlStation.getBackButton() && controlStation.getBackButton()){
      climbEngaged = !climbEngaged;
    }

    prevclimbEngaged = controlStation.getBackButton();
    
    return climbEngaged;
    // return this.climbTable.getEntry("isClimbing").getBoolean(false);
  }

  @Override
  public double climbVelocityL() {
    return this.climbTable.getEntry("climbVelocityL").getDouble(0.0);
  }

  @Override
  public double climbVelocityR() {
    return this.climbTable.getEntry("climbVelocityR").getDouble(0.0);
  }


  @Override
  public boolean driveIsAutoRotating() {
    return xbox.getLeftTriggerAxis() > 0.5;
  }
}