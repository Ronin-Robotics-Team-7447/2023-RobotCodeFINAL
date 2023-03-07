package com.team6560.frc2023.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.team6560.frc2023.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class Telescope extends SubsystemBase {
    GenericHID armJoystick;
    TalonSRX m_telescope;
    Encoder telescopeEncoder;

    boolean wedonotwantogoupanymore = false;
    boolean wedonotwantogodownanymore = false;
    boolean resetingEncoders = false;

    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("telescope");
    NetworkTableEntry upperLimitNT = networkTable.getEntry("Telescope Upper Limit");
    NetworkTableEntry bottomLimitNT = networkTable.getEntry("Telescope Lower Limit");
    NetworkTableEntry resetEncoders = networkTable.getEntry("Telescope Reset Encoders");
    NetworkTableEntry currentTelescopePosition = networkTable.getEntry("Telescope Current Position");

    public Telescope() {
        armJoystick = new GenericHID(Constants.TelescopeConstants.logitechID);

        telescopeEncoder = new Encoder(2, 1);
        m_telescope = new TalonSRX(Constants.TelescopeConstants.telescopeID);
        //m_telescope.configPeakCurrentLimit(1);
        //m_telescope.configPeakCurrentDuration(0);

        m_telescope.configFactoryDefault(0);
        m_telescope.setInverted(false);
        
        m_telescope.setNeutralMode(NeutralMode.Brake);
        m_telescope.getFaults(new Faults());

        upperLimitNT.setDouble(Constants.TelescopeConstants.upperLimit);
        bottomLimitNT.setDouble(Constants.TelescopeConstants.bottomLimit);
        resetEncoders.setBoolean(false);
        currentTelescopePosition.setDouble(0);
    }
    @Override
    public void periodic() {
        currentTelescopePosition.setDouble(telescopeEncoder.getRaw());
        if( resetingEncoders == true && resetEncoders.getBoolean(false) == false ) {
            // Set Encoders to 0;
            telescopeEncoder.reset();
            resetingEncoders = false;
        } else if ( resetingEncoders == false && resetEncoders.getBoolean(false) == false ) {
            passedLimits(bottomLimitNT.getDouble(0), upperLimitNT.getDouble(0));
            this.moveTelescope(-armJoystick.getRawAxis(Constants.TelescopeConstants.throttleAxis));
        } else if( resetingEncoders == true && resetEncoders.getBoolean(false) == true ) {
            passedLimits(-2000, 2000);
            this.moveTelescope(-armJoystick.getRawAxis(Constants.TelescopeConstants.throttleAxis));
        }
        resetingEncoders = resetEncoders.getBoolean(false);
    }

    public void moveTelescope(double telescopeSpeed) {
        if(telescopeSpeed > 0.2) {
          if (wedonotwantogoupanymore) {
            if( -armJoystick.getRawAxis(Constants.TelescopeConstants.throttleAxis) < 0.0 ) {
              m_telescope.set(TalonSRXControlMode.PercentOutput, Constants.TelescopeConstants.telescopeSpeed);
            } else {
              m_telescope.set(TalonSRXControlMode.PercentOutput,0);
            }
          } else {
            m_telescope.set(TalonSRXControlMode.PercentOutput, Constants.TelescopeConstants.telescopeSpeed);
          }
        } else if(telescopeSpeed < -0.2) {
          if(wedonotwantogodownanymore) {
            if( -armJoystick.getRawAxis(Constants.TelescopeConstants.throttleAxis) > 0.0 ) {
              m_telescope.set(TalonSRXControlMode.PercentOutput, Constants.TelescopeConstants.telescopeSpeed);
            } else {
              m_telescope.set(TalonSRXControlMode.PercentOutput,0);
            }
          } else {
            m_telescope.set(TalonSRXControlMode.PercentOutput, -Constants.TelescopeConstants.telescopeSpeed);
          } 
        } else {
          m_telescope.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }

    public void passedLimits(double lowerLimit, double upperLimit) {
        int currentDistance = telescopeEncoder.getRaw();
        if( currentDistance >= upperLimit ) {
          wedonotwantogoupanymore = true;
          wedonotwantogodownanymore = false;
        } else if( currentDistance <= lowerLimit ) {
          wedonotwantogodownanymore = true;
          wedonotwantogoupanymore = false;
        } else if( currentDistance <= upperLimit && currentDistance >= lowerLimit ) {
          wedonotwantogodownanymore = false;
          wedonotwantogoupanymore = false;
        }
    }
}
