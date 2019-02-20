/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmCommand;



/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.armMotorPort);

 
  private int targetPosition = 0;
 
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualArmCommand());
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kPIDLoopIdx, RobotMap.kTimeoutMs);
		armMotor.setSensorPhase(false);
		armMotor.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.kTimeoutMs);
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.kTimeoutMs);

		/* set the peak and nominal outputs */
		armMotor.configNominalOutputForward(0, RobotMap.kTimeoutMs);
		armMotor.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
		armMotor.configPeakOutputForward(1, RobotMap.kTimeoutMs);
		armMotor.configPeakOutputReverse(-1, RobotMap.kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		armMotor.selectProfileSlot(RobotMap.kSlotIdx, RobotMap.kPIDLoopIdx);
		armMotor.config_kF(0, 0.2, RobotMap.kTimeoutMs);
		armMotor.config_kP(0, 1.0, RobotMap.kTimeoutMs);
		armMotor.config_kI(0, 0, RobotMap.kTimeoutMs);
		armMotor.config_kD(0, 0, RobotMap.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		armMotor.configMotionCruiseVelocity(15000, RobotMap.kTimeoutMs);
		armMotor.configMotionAcceleration(6000, RobotMap.kTimeoutMs);
		/* zero the sensor */
		armMotor.setSelectedSensorPosition(0, RobotMap.kPIDLoopIdx, RobotMap.kTimeoutMs);

  }


  public void manualArm(double speed){
    armMotor.set(ControlMode.PercentOutput, speed);
  }


  public void motionMagicArm(){
    armMotor.set(ControlMode.MotionMagic, targetPosition);
  }

  public void setTargetPosition(int position){
    targetPosition = position;
  }

  public int getArmPosition() {
    return this.armMotor.getSelectedSensorPosition(0);
    
  }

}
