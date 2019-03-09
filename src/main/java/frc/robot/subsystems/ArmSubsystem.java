/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
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
  private int maxVelocity = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualArmCommand());

    armMotor.configFactoryDefault();

    armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
													                  LimitSwitchNormal.NormallyOpen,
                                            RobotMap.kTimeoutMs);
    
    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
													                  LimitSwitchNormal.NormallyOpen,
													                  RobotMap.kTimeoutMs);
    
    

    
    //Configure Talon to clear sensor position on Reverse  Limit "1 clears it 0 does not"
    armMotor.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0, 10);
     //Sets up the encoder to be absolute version of the ctre mag encoder
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, RobotMap.kPIDLoopIdx, RobotMap.kTimeoutMs);
    
    //Sets encoder phase this should be where the encoder counter goes up when the talon lights are green and down when lights are red
    //switch the boolan if it is counting backwards.  Motion magic will go haywire if this is not correct    
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
		armMotor.config_kF(0, 0.89, RobotMap.kTimeoutMs);
		armMotor.config_kP(0, 6.0, RobotMap.kTimeoutMs);
		armMotor.config_kI(0, 0, RobotMap.kTimeoutMs);
		armMotor.config_kD(0, 0, RobotMap.kTimeoutMs);
  
    /* set acceleration and vcruise velocity - see documentation */
		armMotor.configMotionCruiseVelocity(1200, RobotMap.kTimeoutMs);
		armMotor.configMotionAcceleration(1000, RobotMap.kTimeoutMs);
  
    /* zero the sensor */
    // for an absolute sensor this will not actually be zero, but the current location
    armMotor.setSelectedSensorPosition(0, RobotMap.kPIDLoopIdx, RobotMap.kTimeoutMs);
    
    
    //Use these settings to turn on/off software limits and set the limits
    armMotor.configForwardSoftLimitEnable(false);
    armMotor.configForwardSoftLimitThreshold(RobotMap.forwardLimit);

    armMotor.configReverseSoftLimitEnable(false);
    armMotor.configReverseSoftLimitThreshold(RobotMap.reverseLimit);

  }

  //This is to manually drive the arm with an input speed
  public void manualArm(double speed){
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  //This is to use motion magic to drive the arm.  It used the golbal targetPosition variable.  If you want to change the position
  // the setTargetPositoin() method must be used
  public void motionMagicArm(){
    armMotor.set(ControlMode.MotionMagic, targetPosition);
  }

  //This sets the value of the golbal variable targetPosition
  public void setTargetPosition(int position){
    targetPosition = position;
  }

  //This gets the encoder value that tell the arm position
  public int getArmPosition() {
    return this.armMotor.getSelectedSensorPosition(0);

  }
  //This gets the absolute position of the arm
  public int getAbsolutePosition(){
    
    return this.armMotor.getSensorCollection().getPulseWidthPosition();
  }
  //This gets the velocity of the arm
  public int getArmVelocity(){
    return this.armMotor.getSelectedSensorVelocity(0);
  }

  public int getMaxVelocity(){
    int currentVelocity = getArmVelocity();
      if(currentVelocity>maxVelocity){
        maxVelocity=currentVelocity;
      }
    return maxVelocity;
  }

  public boolean isForwardLimit(){
    return armMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean isReverseLimit(){
    return armMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  //This zeros the arm sensor.  When used in absolute mode it will not truly zero, but will give the absolute value and remove any
  //large number due to multiple turns
  public void zeroSensor(){
    armMotor.setSelectedSensorPosition(0, RobotMap.kPIDLoopIdx, RobotMap.kTimeoutMs);
    

  }
}
