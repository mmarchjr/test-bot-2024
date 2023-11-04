// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.utils.SwerveUtils;
import frc.robot.RobotContainer;

public class driverobot extends Command {
  /** Creates a new driverobot. */
  double angle = 0;

  double deadzone = 0.3; // variable for amount of deadzone
  double y = 0; // variable for forward/backward movement
  double x = 0; // variable for side to side movement
  double turn = 0; // variable for turning movement
  PIDController pid = new PIDController(0.0004, 0, 0);
  
  
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public driverobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(5);
    angle = RobotContainer.m_robotDrive.getHeading();
    pid.enableContinuousInput(-180,180);
    
  }
private void Turntoangle() {
  int setangle = m_driverController.getPOV();
    double gyroAngle = RobotContainer.m_robotDrive.getHeading();

    // Ensure the angle wraps around from -180 to 180 degrees
    //gyroAngle = (gyroAngle + 180.0) % 360.0 - 180.0;
           RobotContainer.m_robotDrive.drive(
              0, // forwards(divided by 10)
              0, // sideways(divided by 10)
              -pid.calculate(gyroAngle,setangle), // rotation(divided by 10)
              false,
              RobotContainer.ratelimitChooser.getSelected() // limit max speed
              );
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = 0;
    y = 0;
    turn = 0;
SmartDashboard.putNumber("gyro", RobotContainer.m_robotDrive.getHeading());
    if (m_driverController.getRightY() > deadzone || m_driverController.getRightY() < -deadzone) {
      y = m_driverController.getRightY();
    }

    if (m_driverController.getRightX() > deadzone || m_driverController.getRightX() < -deadzone) {
      x = m_driverController.getRightX();
    }

    if (m_driverController.getLeftX() > deadzone || m_driverController.getLeftX() < -deadzone) {
      turn = m_driverController.getLeftX();
    }
    /*if(m_driverController.getXButton()){
        y=0;
        x=0;
        turn=0;
        RobotContainer.m_robotDrive.setX();
    }*/

    
    
      if (m_driverController.getPOV() != -1) {
        Turntoangle();
       //new RunCommand(() -> Turntoangle(), RobotContainer.m_robotDrive).until(() -> pid.atSetpoint());
// Ensure the angle wraps around from -180 to 180 degrees
    } else if (x==0 && y==0 && turn==0){
      RobotContainer.m_robotDrive.setX();
    } else {
      RobotContainer.m_robotDrive.drive(
          -y / 2, // forwards(divided by 10)
          -x / 2, // sideways(divided by 10)
          -turn/2, // rotation(divided by 10)
          RobotContainer.fieldoriented.getSelected(), // field oriented
          RobotContainer.ratelimitChooser.getSelected() // limit max speed
          );
    }

    if (m_driverController.getYButton()) {
      RobotContainer.m_robotDrive.resetGyro();
      m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
    }
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
