// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.WaitBehavior;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Claw;
import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * 
 * The AutoBuilder class generates commands for autonomous actions using a
 * Drivetrain and path planning.
 * 
 * It uses a HashMap to store "marker" events and corresponding actions, as well
 * as a SwerveAutoBuilder for generating path following commands.
 */
public class AutoBuilder {

  /**
   * 
   * A HashMap containing "marker" events and corresponding actions.
   */
  private HashMap<String, Command> eventMap;
  
  /**
   * 
   * A list of PathPlannerTrajectory objects that represent the group of paths for
   * autonomous action.
   */
  private List<PathPlannerTrajectory> pathGroup;

  /**
   * 
   * An instance of SwerveAutoBuilder used to generate path following commands.
   */
  private SwerveAutoBuilder autoBuilder;

  /**
   * 
   * An instance of Drivetrain that represents the drive subsystem.
   */
  private Drivetrain drivetrain;
  private SwerveAutoBuilder teleopAutoBuilder;
  private Claw claw;
  /**
   * 
   * Constructor for AutoBuilder class. Initializes eventMap, autoBuilder, and drivetrain.
   * 
   * @param drivetrain An instance of Drivetrain that represents the drive ubsystem.
   */
  public AutoBuilder(Drivetrain drivetrain, Claw claw) {
    this.drivetrain = drivetrain;
    this.claw = claw;

    eventMap = new HashMap<>();
    eventMap.put("printCommand", new PrintCommand("test Print command"));

    autoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getPose(), // Pose2d supplier
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.95), // PID constants to correct for translation error (used to create the X and Y ID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive subsystem
        eventMap,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );
    teleopAutoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getPose(), // Pose2d supplier
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.95), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.6, 0.1, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                          // subsystem
        eventMap,
        false,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );
  }

  /**
   * 
   * Returns a command for autonomous action based on the specified path name.
   * 
   * @param pathName The name of the path file to be loaded and followed.
   * 
   * @return Command for autonomous action based on the specified path.
   */
  public Command getAutoCommand(String pathName) {
    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 2.0 m/s and a max acceleration of 1.0 m/s^2 for every path in the group
    pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(0.75, 0.35));

    return autoBuilder.fullAuto(pathGroup);
  }

  /**
   * 
   * Returns a command for autonomous action to go to a specified Pose2d.
   * 
   * @param desiredPose The desired final position and orientation for the robot to reach.
   * 
   * @return Command for autonomous action to go to the specified Pose2d.
   */
  public Command goToPose(Pose2d desiredPose, Rotation2d heading) {

    Pose2d currPose = drivetrain.getPose();

    ChassisSpeeds currChassisSpeeds = drivetrain.getChassisSpeeds();

    double currSpeed = Math.abs(Math.hypot(currChassisSpeeds.vxMetersPerSecond, currChassisSpeeds.vyMetersPerSecond));

    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(0.5, 0.3),
        // position, heading(direction of travel), holonomic rotation, velocity verride
        new PathPoint(currPose.getTranslation(), heading, currPose.getRotation(), currSpeed),
        new PathPoint(desiredPose.getTranslation(), heading, desiredPose.getRotation()));

    return teleopAutoBuilder.followPath(traj);
  }

  public Command getPathStraightWithIntakeOut() {
    pathGroup = PathPlanner.loadPathGroup("ScoreSideStraight", new PathConstraints(1.5, 0.75));
    return new SequentialCommandGroup(
      new EnableIntake(claw).withTimeout(2),
      autoBuilder.fullAuto(pathGroup)
    );
  }

  public Command getMidAround() {
    pathGroup = PathPlanner.loadPathGroup("MidAround1", new PathConstraints(1.5, 0.75));
    drivetrain.resetOdometry(pathGroup.get(0).getInitialHolonomicPose());

    double offsetRoll = drivetrain.getRoll().getDegrees();
    double offsetPitch = drivetrain.getPitch().getDegrees();
    return new SequentialCommandGroup(
      autoBuilder.fullAuto(pathGroup),
      new ChargingStationAuto(drivetrain, offsetPitch, offsetRoll)
    );
  }

  public Command getStraight() {
    pathGroup = PathPlanner.loadPathGroup("Straight", new PathConstraints(0.8, 0.6));
    return new SequentialCommandGroup(
      autoBuilder.fullAuto(pathGroup)
    );
  }

  public Command getAutoBalanceCommand() {
    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 2.0 m/s and a max acceleration of 1.0 m/s^2 for every path in the group
    pathGroup = PathPlanner.loadPathGroup("ChargingStationAuto", new PathConstraints(0.8, 0.6));

    drivetrain.resetOdometry(pathGroup.get(0).getInitialHolonomicPose());

    double offsetRoll = drivetrain.getRoll().getDegrees();
    double offsetPitch = drivetrain.getPitch().getDegrees();
    return new SequentialCommandGroup(
      new EnableIntake(claw).withTimeout(2),
      autoBuilder.fullAuto(pathGroup),
      new ChargingStationAuto(drivetrain, offsetPitch, offsetRoll)
    );
  }

}
