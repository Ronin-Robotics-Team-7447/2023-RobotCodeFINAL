// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023;
// import java.io.File;

import com.team6560.frc2023.commands.DriveCommand;
import com.team6560.frc2023.commands.IntakeCommand;
import com.team6560.frc2023.commands.MoveClawManually;
import com.team6560.frc2023.commands.MoveWristManually;
import com.team6560.frc2023.commands.SwitchWristLimits;
import com.team6560.frc2023.commands.auto.AutoBuilder;
import com.team6560.frc2023.controls.ManualControls;
import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Drivetrain;
import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Limelight;
import com.team6560.frc2023.subsystems.Telescope;
import com.team6560.frc2023.subsystems.Wrist;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
        Joystick armJoystick;

        Trigger POV0button;
        Trigger POV180button;

        Trigger button1;
        Trigger button2;
        Trigger button3;
        Trigger button4;
        Trigger button5;
        Trigger button6;
        Trigger button7;
        Trigger button8;
        Trigger button9;
        Trigger button10;
        Trigger button11;
        Trigger button12;

        Wrist m_wrist;
        MoveClawManually intake;
        MoveClawManually outtake;
        MoveWristManually moveWristUp;
        MoveWristManually moveWristDown;
        SwitchWristLimits disableWristLimits;
        SwitchWristLimits enableWristLimits;
        // The robot's subsystems and commands are defined here...

        // not public or private so Robot.java has access to it.
        Drivetrain drivetrain = null;

        private final Limelight limelight;

        private final DriveCommand driveCommand;

        //private final Intake intake;

        //private final Arm arm;

        //private final Telescope telescope;

        private final ManualControls manualControls = new ManualControls(new XboxController(0), new XboxController(1));


        // A chooser for autonomous commands
        private final SendableChooser<String> autoChooser;

        private final AutoBuilder autoBuilder;

        private IntakeCommand intakeCommand;

        private  AddressableLED led;


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_wrist = new Wrist();
                intake = new MoveClawManually(m_wrist, Constants.WristConstants.clawSpeed);
                outtake = new MoveClawManually(m_wrist, -Constants.WristConstants.clawSpeed);
                moveWristUp = new MoveWristManually(m_wrist, 0.3, 1);
                moveWristDown = new MoveWristManually(m_wrist, -0.3, -1);
                disableWristLimits = new SwitchWristLimits(m_wrist, true);
                enableWristLimits = new SwitchWristLimits(m_wrist, false);

                limelight = new Limelight(manualControls, () -> drivetrain == null ? null : drivetrain.getPose());

                drivetrain = new Drivetrain(() -> limelight.getBotPose());
                
                //telescope = new Telescope();
                //arm = new Arm();

                autoBuilder = new AutoBuilder(drivetrain);

                driveCommand = new DriveCommand(drivetrain, autoBuilder, limelight, manualControls);
                drivetrain.setDefaultCommand(driveCommand);

                // intake = new Intake();
                // intakeCommand = new IntakeCommand(intake, manualControls);
                // intake.setDefaultCommand(intakeCommand);


                autoChooser = new SendableChooser<String>();

                // Add commands to the autonomous command chooser
                String defaultAuto = "Straight";
                autoChooser.setDefaultOption(defaultAuto, defaultAuto);

                // for (String f : (new File(Filesystem.getDeployDirectory().getPath() + "/pathplanner")).list()) {
                //         f = f.strip().replace(".path", "");
                //         if (!f.equals(defaultAuto)) {
                //                 autoChooser.addOption(f, f);
                //                 System.out.println(f);
                //         }
                // }

                autoChooser.addOption("StraightSpin", "StraightSpin");
                autoChooser.addOption("StraightAndBackSpin", "StraightAndBackSpin");
                autoChooser.addOption("StraightAndBack", "StraightAndBack");
                autoChooser.addOption("Hamid", "Hamid");
                autoChooser.addOption("HamidSharp", "HamidSharp");
                autoChooser.addOption("ChargingStationAuto", "ChargingStationAuto");
                autoChooser.addOption("NewPath", "NewPath");

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Auto Choose").add(autoChooser);

                led = new AddressableLED(3);

                armJoystick = new Joystick(Constants.TelescopeConstants.logitechID);

                POV0button = new POVButton(armJoystick, Constants.ArmJoystickConstants.buttonPOV0);
                POV180button = new POVButton(armJoystick, Constants.ArmJoystickConstants.buttonPOV180);
            
                button1 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button1);
                button2 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button2);
                button3 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button3);
                button4 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button4);
                button5 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button5);
                button6 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button6);
                button7 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button7);
                button8 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button8);
                button9 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button9);
                button10 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button10);
                button11 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button11);
                button12 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button12);

                turnLightsOn();
                configureBindings();
        }


        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoBuilder.getAutoCommand(autoChooser.getSelected());
        }

        public void turnLightsOn() {
                AddressableLEDBuffer ledbuffer = new AddressableLEDBuffer(0);
                led.setLength(ledbuffer.getLength());
                for (var i = 0; i < ledbuffer.getLength(); i++) {
                        // Sets the specified LED to the RGB values for red
                        ledbuffer.setRGB(i, 50, 147, 168);
                }
                led.setData(ledbuffer);
                led.start();
        }


        private void configureBindings() {
                // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
                // new Trigger(m_exampleSubsystem::exampleCondition)
                //     .onTrue(new ExampleCommand(m_exampleSubsystem));
            
                // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
                // // cancelling on release.
                // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
            
                POV0button.whileTrue(moveWristUp);
                POV180button.whileTrue(moveWristDown);
            
                // // RESERVED FOR INTAKE AND OUTTAKE
                button1.whileTrue(intake);
                button2.whileTrue(outtake);
            
                // RESERVED FOR PRESET POSITIONS OF ENTIRE ARM
                // Right hand buttons
                button3.toggleOnTrue(disableWristLimits).toggleOnFalse(enableWristLimits);
                // button4.onTrue(null);
                // button5.onTrue(null);
                // button6.onTrue(null);
            
                // // Left hand buttons
                // button7.onTrue(null);
                // button8.onTrue(null);
                // button9.onTrue(null);
                // button10.onTrue(null);
                // button11.onTrue(null);
                // button12.onTrue(null);
              }
}
