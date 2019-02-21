/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.SetMotionMagicArmCommand;
import frc.robot.commands.dashBoardSetMotionMagicArmCommand;
import frc.robot.commands.ZeroSensorCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  public Joystick joystick = new Joystick(RobotMap.joystickPort);
  Button xButton = new JoystickButton(joystick, 1);
  Button aButton = new JoystickButton(joystick, 2);

  public OI(){
    SmartDashboard.putData("ManualArm", new ManualArmCommand());
    SmartDashboard.putData("Motion Magic Arm", new dashBoardSetMotionMagicArmCommand());
    SmartDashboard.putData("Zero Sensor", new ZeroSensorCommand());
    SmartDashboard.putData("Arm Up", new SetMotionMagicArmCommand(0));
    SmartDashboard.putData("Arm Down", new SetMotionMagicArmCommand(-1260));

    xButton.whenPressed(new SetMotionMagicArmCommand(0));
    aButton.whenPressed(new SetMotionMagicArmCommand(-1260));


    



  }
  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
