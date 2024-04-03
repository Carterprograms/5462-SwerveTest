// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.hardware.TalonFX;

// This module should contain a:
// 1. Drive Motor
// 2. Turning Motor
// 3. Absolute Encoder

class SwerveModule {
  // Define varibles needed
 TalonFX driveMotor;
 TalonFX steeringMotor;

 // State Varibles
  SwerveModuleState currentState;
  SwerveModuleState desiredState;

 // Two PID Controllers needed
 PIDController drivePIDController;
 PIDController steeringPIDController;

  public SwerveModule(
    int driveMotorPort,
    int turningMotorPort
  ) {
      System.out.println("SwerveModule constructor");
      driveMotor = new TalonFX(driveMotorPort);
      steeringMotor = new TalonFX(turningMotorPort);

      // Initialize PID's
      drivePIDController = new PIDController(0, 0, 0);
      steeringPIDController = new PIDController(0, 0, 0);
  }
   // Simulation
      FlywheelSim steeringSim = new FlywheelSim(
        DCMotor.getKrakenX60(8),
        150.0 / 7.0,
        0.004
        );
  public SwerveModuleState getState() {
    return currentState;
  }

  public void setDesiredState(SwerveModuleState newState) {
    desiredState = newState;
  }

  public void periodic() {
    // Run the sim
    steeringSim.update(0.02);

    // Get the new simulated angles
    double simulatedAngleDiffRad = steeringSim.getAngularVelocityRadPerSec() * 0.02;

    // Update current state
    currentState = new SwerveModuleState(
    // Set new speed
    currentState.speedMetersPerSecond,
    // Set new angle 
    Rotation2d.fromDegrees(currentState.angle.getDegrees() + simulatedAngleDiffRad)
    );
  }
}

public class SwerveDriveSubsystem extends SubsystemBase {

// List of swerve modules.
  SwerveModule frontLeftModule = new SwerveModule(1, 2);
  SwerveModule frontRightModule = new SwerveModule(3, 4);
  SwerveModule backLeftModule = new SwerveModule(5, 6);
  SwerveModule backRightModule = new SwerveModule(7, 8);


  // Define kinematics (Converts chassis speed into module speeds)
  double chassisWidth = Units.inchesToMeters(32);
  double chassisLength = Units.inchesToMeters(32);

  // Locations of modules based off center of robot
  Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
  Translation2d frontRightLocation = new Translation2d(chassisLength / 2, -chassisWidth / 2);
  Translation2d backLeftLocation = new Translation2d(-chassisLength/ 2, chassisWidth /2);
  Translation2d backRightLocation = new Translation2d(-chassisLength / 2, -chassisWidth /2);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation,
    frontRightLocation,
    backLeftLocation,
    backRightLocation
  );

  // Get a refrence to the contoller
  CommandXboxController controller;

  // Constructor
  public SwerveDriveSubsystem(CommandXboxController io) {
    System.out.println("SwerveDriveSubsystem constructor");
    controller = io;
  }


    public void setChassisSpeed(ChassisSpeeds desired) {
      // Desired states of wheels
      SwerveModuleState [] newStates = kinematics.toSwerveModuleStates(desired);
     
      // Set sate of modules (speed/direction)
      frontLeftModule.setDesiredState(newStates[0]);
      frontRightModule.setDesiredState(newStates[1]);
      backLeftModule.setDesiredState(newStates[2]);
      backRightModule.setDesiredState(newStates[3]);

    }
    @Override
    public void periodic() {

      // Read data from the controller
      // Get the x and y values of the left joystick
      ChassisSpeeds newDesiredSpeed = new ChassisSpeeds(
        // Forward joystick = Forward Bot
        controller.getLeftY(),
        // Left joystick = Left Bot
        controller.getLeftX(),
        // Right joystick = Bot Rotate Left
        controller.getRightX()
      );

      System.out.println(newDesiredSpeed);

      // Order = FL, FR, BL, BR
      double loggingState[] = {
        // Module 1
        frontLeftModule.getState().angle.getDegrees(),
        frontLeftModule.getState().speedMetersPerSecond,

        frontRightModule.getState().angle.getDegrees(),
        frontRightModule.getState().speedMetersPerSecond,

        backLeftModule.getState().angle.getDegrees(),
        backLeftModule.getState().speedMetersPerSecond,
        
        backRightModule.getState().angle.getDegrees(),
        backRightModule.getState().speedMetersPerSecond
    };

    setChassisSpeed(newDesiredSpeed);

    SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);

   }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
