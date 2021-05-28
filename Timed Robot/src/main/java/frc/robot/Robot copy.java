// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import org.ejml.equation.VariableScalar;

import edu.wpi.first.wpilibj.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private Spark leftMotor1 = new Spark (0);
  private Spark leftMotor = new Spark(1);
  private Spark rightMotor1 = new Spark(2);
  private Spark leftMotor2 = new Spark(3);

  private Joystick joy1 = new Joystick(0);

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4x);
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;


  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    encoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimeStamp = Timer.getFPGATimsestamp();
  }
  
  final double kP = 0.5;
  final double kI = 5;
  final double kD = 0.01;
  final double iLimit = 1;

  double setpoint = 0; 
  double errorSum = 0;
  double lastTimeStamp = 0;
  double lastError = 0;

  @Override
  public void autonomousPeriodic() {
    // get Joystick command
    if (joy1.getRawButton(1)) {
      setpoint = 10;
    } else if (joy1.getRawButton(2)) {
      setpoint = 0;
    }

    // get sensor position
    double sensorPosition = encoder.get() * kDriveTick2Feet;

    //calculations
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    if (Math.abs(error) < iLimit) {
     errorSum += error * dt;
    }

    double errorrate = (error - lastError) / dt;

    double outputSpeed = kP * error + kI * errorSum + kD * error;

    // output to motors
    leftMotor1.set(outputSpeed);
    rightMotor2.set(outputSpeed);
    leftMotor1.set(-outputSpeed);
    rightMotor2.set(-outputSpeed);

    //update last- variables
    lastTimeStamp=  Timer.getFPGATimestamp()
    lastError = error;
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder Value", encoder.get()*kDriveTick2Feet);
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
