// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(1);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(2);

  private WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
  private WPI_VictorSPX armSlave = new WPI_VictorSPX(3);

  private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(4);

  private Compressor compressor = new Compressor();
  private DoubleSolenoid hatchIntake = new DoubleSolenoid(0, 1); // PCM port 0, 1

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // Joysticks
  private Joystick driverJoystick = new Joystick(0);
  private Joystick operatorJoystick = new Joystick(1);

  //unit conversion
  private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
  private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;

@Override
public void robotPeriodic() {
  SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition() * kArmTick2Deg);
  SmartDashboard.putNumber("Left Drive Encoder Valeue", leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
  SmartDashboard.putNumber("Right Dirve Encoder Valeue", rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);
}  


   

  @Override
  public void robotInit() {
      //inverted settings
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    armMotor.setInverted(false);

    //slave setups 
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    armSlave.follow(armMotor);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    armSlave.setInverted(InvertType.FollowMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    armSlave.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);
    armMotor.setSesnorPhase(true);

    // set encoders back to zero
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorposition(0, 0, 10);

    //set encoder boundrary limits: to stop Motors
    armMotor.configReverseSoftLimitThreshold((int) (0 / kArmTick2Deg), 10);
    armMotor.configReverseSoftLimitThreshold((int) (175 / kArmTick2Deg), 10);

    armMotor.configReverseSoftLimitThreshold(true, 10);
    armMotor.configReverseSoftLimitThreshold(true, 10);

    //start Compressor
    compressor.start();

    drive.setDeadband(0.05);
  }

  @Override
  public void autonomousInit() {
    enableMotors(true);
    // set encoders back to zero
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorposition(0, 0, 10);
  }

  @Override
  public void autonomousPeriodic() {
    double leftPosition = leftMaster.setSelectedSensorposition() * kDriveTick2Feet;
    double rightPosition = rightMaster.setSelectedSensorposition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;

    if (distance < 10) {
      drive.tankDrive(0,0);
    }
  }

  @Override
  public void teleopInit() {
    enableMotors(true);
  }
  
  private void compressor() {
  }

  public Joystick getOperatorJoystick() {
    return operatorJoystick;
  }

  public void setOperatorJoystick(Joystick operatorJoystick) {
    this.operatorJoystick = operatorJoystick;
  }

  public Joystick getDriverJoystick() {
    return driverJoystick;
  }

  public void setDriverJoystick(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;
  }

  public DoubleSolenoid getHatchIntake() {
    return hatchIntake;
  }

  public void setHatchIntake(DoubleSolenoid hatchIntake) {
    this.hatchIntake = hatchIntake;
  }

  public Compressor getCompressor() {
    return compressor;
  }

  public void setCompressor(Compressor compressor) {
    this.compressor = compressor;
  }

  public WPI_TalonSRX getRollerMotor() {
    return rollerMotor;
  }

  public void setRollerMotor(WPI_TalonSRX rollerMotor) {
    this.rollerMotor = rollerMotor;
  }

  public WPI_VictorSPX getArmSlave() {
    return armSlave;
  }

  public void setArmSlave(WPI_VictorSPX armSlave) {
    this.armSlave = armSlave;
  }

  public WPI_TalonSRX getArmMotor() {
    return armMotor;
  }

  public void setArmMotor(WPI_TalonSRX armMotor) {
    this.armMotor = armMotor;
  }

  public WPI_VictorSPX getRightSlave() {
    return rightSlave;
  }

  public void setRightSlave(WPI_VictorSPX rightSlave) {
    this.rightSlave = rightSlave;
  }

  public WPI_VictorSPX getLeftSlave() {
    return leftSlave;
  }

  public void setLeftSlave(WPI_VictorSPX leftSlave) {
    this.leftSlave = leftSlave;
  }

  public WPI_TalonSRX getRightMaster() {
    return rightMaster;
  }

  public void setRightMaster(WPI_TalonSRX rightMaster) {
    this.rightMaster = rightMaster;
  }

  public WPI_TalonSRX getLeftMaster() {
    return leftMaster;
  }

  public void setLeftMaster(WPI_TalonSRX leftMaster) {
    this.leftMaster = leftMaster;
  }

  @Override
  public void teleopPeriodic() {
    // driving
    double power = -driverJoystick.getRawAxis(1); // remember: negative sign
    double turn = driverJoystick.getRawAxis(4); 
    // deadband
    if (Math.abs(power) < 0.05) {
      power = 0;
    }
    if (Math.abs(turn) < 0.05) {
      turn = 0;
    }
    drive.arcadeDrive(power * 0.6, turn * 0.3);

    // arm control
    double armPower = - operatorJoystick.getRawAxis(1); // remember negative sign
    if (Math.abs(armPower) < 0.05) {
         armPower = 0;
    }
    armMotor *= 0.5;
    armMotor.set(ControlMode.PercentOutput, armPower);

    //roller control
    double rollerPower = 0;
    if (operatorJoystick.getRawButton(1)==true){
      rollerPower = 1;
    } else if (operatorJoystick.getRawButton(2)) {
      rollerPower = -1;
    }
    rollerMotor.set(ControlMode.Percentoutput, rollerPower);

    // Hatch intake
    if (operatorJoystick.getRawButton(3)) {// open
     hatchIntake.set(Value.kReverse);
    } else {
      hatchIntake.set(Value.kForward);
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    enableMotors(false);
  }

  private void enableMotors(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake
    }else{
      mode = NeutralMode.Coast;
    }
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftSlave.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);
    armMotor.setNeutralMode(mode);
    armSlave.setNeutralMode(mode);
    rollerMotor.setNeutralMode(mode);
  }

}
