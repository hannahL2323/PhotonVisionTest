// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("photonvision");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  XboxController xboxController = new XboxController(0);

  // Drive motors

//   CANSparkMax Left1 = new CANSparkMax(0, MotorType.kBrushless);
//   CANSparkMax Left2 = new CANSparkMax(1, MotorType.kBrushless);

  WPI_TalonSRX Left1 = new WPI_TalonSRX(1);
  WPI_TalonSRX Left2 = new WPI_TalonSRX(2);
  WPI_TalonSRX Right1 = new WPI_TalonSRX(4);
  WPI_TalonSRX Right2 = new WPI_TalonSRX(3);


  MotorControllerGroup leftGroup = new MotorControllerGroup(Left1, Left2);
  MotorControllerGroup rightGroup = new MotorControllerGroup(Right1, Right2);


  DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  @Override
  public void teleopPeriodic() {
      double forwardSpeed;
      double rotationSpeed;

      forwardSpeed = -xboxController.getRightY();

      if (xboxController.getAButton()) {
          // Vision-alignment mode
          // Query the latest result from PhotonVision
          var result = camera.getLatestResult();

          if (result.hasTargets()) {
              // Calculate angular turn power
              // -1.0 required to ensure positive PID controller effort _increases_ yaw
              rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
          } else {
              // If we have no targets, stay still.
              rotationSpeed = 0;
          }
      } else {
          // Manual Driver Mode
          rotationSpeed = xboxController.getLeftX();
      }

      // Use our forward/turn speeds to control the drivetrain
      drive.arcadeDrive(forwardSpeed, rotationSpeed);
  }
}