// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.utils.RoaringUtils;
import frc.utils.SwerveUtils;
import frc.robot.RobotContainer;

public class driverobot extends Command {
  /** Creates a new driverobot. */
  double angle = 0;

  double deadzone = 0.3; // variable for amount of deadzone
  double y = 0; // variable for forward/backward movement
  double x = 0; // variable for side to side movement
  double turn = 0; // variable for turning movement
  PIDController turnController;
  double pastTurn = 0;
  
  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system.  Note that the      */
  /* SmartDashboard in Test mode has support for helping you tune    */
  /* controllers by displaying a form where you can enter new P, I,  */
  /* and D constants and test the mechanism.                         */
  
  static final double kP = 0.02;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 2.0f;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public driverobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(kP, kI, kD);
    turnController.enableContinuousInput(-180.0f,  180.0f);
    turnController.setTolerance(kToleranceDegrees);
    angle = RobotContainer.m_robotDrive.getHeading();

    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = 0;
    y = 0;
    turn = 0;
SmartDashboard.putNumber("gyro", RobotContainer.m_robotDrive.getHeading());
    /*if (m_driverController.getRightY() > deadzone || m_driverController.getRightY() < -deadzone) {
      y = m_driverController.getRightY();
    }

    if (m_driverController.getRightX() > deadzone || m_driverController.getRightX() < -deadzone) {
      x = m_driverController.getRightX();
    }

    if (m_driverController.getLeftX() > deadzone || m_driverController.getLeftX() < -deadzone) {
      turn = m_driverController.getLeftX();
    }*/
    /*if(m_driverController.getXButton()){
        y=0;
        x=0;
        turn=0;
        RobotContainer.m_robotDrive.setX();
    }*/

    y=RoaringUtils.DeadzoneUtils.LinearDeadband(m_driverController.getRightY(),0.1);
    x=RoaringUtils.DeadzoneUtils.LinearDeadband(m_driverController.getRightX(),0.1);
 turn=RoaringUtils.DeadzoneUtils.LinearDeadband(m_driverController.getLeftX(),0.1);
    
      /*if (m_driverController.getPOV() != -1) {
        Turntoangle();
       //new RunCommand(() -> Turntoangle(), RobotContainer.m_robotDrive).until(() -> pid.atSetpoint());
// Ensure the angle wraps around from -180 to 180 degrees
    } else*/ 
    
    
          boolean rotateToAngle = false;
           if (turn != pastTurn) {
              turnController.setSetpoint(RobotContainer.m_robotDrive.getHeading());
          }
          if (m_driverController.getPOV()==0) {
              turnController.setSetpoint(0.0f);
              rotateToAngle = true;
          } else if (m_driverController.getPOV()==90) {
              turnController.setSetpoint(-90.0f);//inverted
              rotateToAngle = true;
          } else if (m_driverController.getPOV()==180) {
              turnController.setSetpoint(179.9f);
              rotateToAngle = true;
          } else if (m_driverController.getPOV()==270) {
              turnController.setSetpoint(90.0f);//inverted
              rotateToAngle = true;
          }
          double currentRotationRate;
          if ( rotateToAngle ) {
              currentRotationRate = MathUtil.clamp(turnController.calculate(RobotContainer.m_robotDrive.getHeading()),-1,1); // rotation(divided by 4)
;
          } else {
            if (turn == 0) {
              currentRotationRate = MathUtil.clamp(turnController.calculate(RobotContainer.m_robotDrive.getHeading()),-1,1);
            } else {
              currentRotationRate = -turn;
            }
          }
          try {
            RobotContainer.m_robotDrive.drive(
          -y / 2, // forwards(divided by 4)
          -x / 2, // sideways(divided by 4)
          currentRotationRate/2,
          RobotContainer.fieldoriented.getSelected(), // field oriented
          RobotContainer.ratelimitChooser.getSelected() // limit max speed
          );
              /* Use the joystick X axis for lateral movement,          */
              /* Y axis for forward movement, and the current           */
              /* calculated rotation rate (or joystick Z axis),         */
              /* depending upon whether "rotate to angle" is active.    */
              //myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(), 
              //                               currentRotationRate, ahrs.getAngle());
          } catch( RuntimeException ex ) {
              DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
          }
          pastTurn = turn;
    /*if (x==0 && y==0 && turn==0){
      RobotContainer.m_robotDrive.setX();
    } else {
      RobotContainer.m_robotDrive.drive(
          -y / 2, // forwards(divided by 4)
          -x / 2, // sideways(divided by 4)
          -turn/2, // rotation(divided by 4)
          RobotContainer.fieldoriented.getSelected(), // field oriented
          RobotContainer.ratelimitChooser.getSelected() // limit max speed
          );
    }*/



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
