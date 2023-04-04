// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Offset;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.Balance;
import frc.robot.commands.BasicBalance;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.CloseGroundControl;
import frc.robot.commands.ControlLights;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ExtendArmWithJoystick;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeWithSwitch;
import frc.robot.commands.LowerArm;
import frc.robot.commands.LowerBoxingGloves;
import frc.robot.commands.LowerGroundControl;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.OpenGroundControl;
import frc.robot.commands.RaiseArm;
import frc.robot.commands.RaiseBoxingGloves;
import frc.robot.commands.RaiseGroundControl;
import frc.robot.commands.ResetArmEncoder;
import frc.robot.commands.LockPosition;
import frc.robot.commands.SetArmHeight;
import frc.robot.commands.StopDrive;
import frc.robot.commands.TurnSlow;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BoxingGloves;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.GroundControl;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.NeoDrive;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Arm _arm = new Arm();  
  private final GroundControl _groundControl = new GroundControl(_arm);
  private final DriveTrain _driveTrain = new NeoDrive(_arm, true); // change to neoDrive for a neo bot
  private final Gripper _gripper = new Gripper();
  private final Lights _lights = new Lights();
  private final Vision _vision = new Vision(true, _arm);
  private final BoxingGloves _boxingGloves = new BoxingGloves();
  private final Joystick _leftDriveJoystick = new Joystick(2);//For tank drive
  private final Joystick _rightDriveJoystick = new Joystick(3);//For tank drive

  private final Joystick _functionJoystick = new Joystick(0);
  private final Joystick _functionJoystick2 = new Joystick(1);

  private final Commands _commands = new Commands(_driveTrain, _groundControl, _arm, _gripper, _vision, _boxingGloves);
  


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _arm.get(_groundControl);
    _driveTrain.setDefaultCommand(new DriveWithJoysticks(_driveTrain, _leftDriveJoystick, _rightDriveJoystick));// for tank drive
    _arm.setDefaultCommand(new ExtendArmWithJoystick(_arm, _functionJoystick2));//Was with buttons
    _groundControl.setDefaultCommand(new IntakeWithSwitch(_groundControl, _functionJoystick, .25));
    _lights.setDefaultCommand(new ControlLights(_lights, _functionJoystick));
    // Configure the trigger bindings
    configureBindings();
    autoChooserInit();
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    JoystickButton one = new JoystickButton(_functionJoystick, 1);
    JoystickButton two = new JoystickButton(_functionJoystick, 2);
    JoystickButton three = new JoystickButton(_functionJoystick, 3); // used by Intake with Switch 
    JoystickButton four = new JoystickButton(_functionJoystick, 4);
    JoystickButton five = new JoystickButton(_functionJoystick, 5); 
    JoystickButton six = new JoystickButton(_functionJoystick, 6); 
    JoystickButton seven = new JoystickButton(_functionJoystick, 7);

    JoystickButton one2 = new JoystickButton(_functionJoystick2, 1);
    JoystickButton two2 = new JoystickButton(_functionJoystick2, 2);
    JoystickButton three2 = new JoystickButton(_functionJoystick2, 3);
    JoystickButton four2 = new JoystickButton(_functionJoystick2, 4);
    JoystickButton five2 = new JoystickButton(_functionJoystick2, 5);
    JoystickButton six2 = new JoystickButton(_functionJoystick2, 6);
    JoystickButton seven2 = new JoystickButton(_functionJoystick2, 7);
    JoystickButton eight2 = new JoystickButton(_functionJoystick2, 8);

    //Function Button Board
    one.whileTrue(new Intake(_groundControl, -.8));
    two.whileTrue(new Intake(_groundControl, .4));

    // four.onTrue(new ControlLights(_lights, _functionJoystick));
    // four.onFalse(new ControlLights(_lights, _functionJoystick));
    // five.onTrue(new ControlLights(_lights, _functionJoystick));
    // five.onFalse(new ControlLights(_lights, _functionJoystick));
    // six.onTrue(new ControlLights(_lights, _functionJoystick));
    // six.onFalse(new ControlLights(_lights, _functionJoystick));
    // seven.onTrue(new ControlLights(_lights, _functionJoystick));
    // seven.onFalse(new ControlLights(_lights, _functionJoystick));

    five.onTrue(new RaiseBoxingGloves(_boxingGloves));
    five.onFalse(new LowerBoxingGloves(_boxingGloves));

    one2.onTrue(new SetArmHeight(_arm, ArmConstants.retracted)
    .andThen(new RaiseArm(_arm)));
    two2.onTrue(new OpenGroundControl(_groundControl)
    .andThen(new WaitCommand(.02))
    .andThen(new LowerArm(_arm)
    .andThen(new SetArmHeight(_arm, ArmConstants.middlePosition))));
    three2.onTrue(new OpenGroundControl(_groundControl)
    .andThen(new WaitCommand(.02))
    .andThen(new LowerArm(_arm)
    .andThen(new SetArmHeight(_arm, ArmConstants.highPosition))));
    four2.onTrue(new RaiseArm(_arm)
    .andThen(new SetArmHeight(_arm, ArmConstants.pickupPosition)));

    five2.onTrue(new OpenGroundControl(_groundControl)
      .andThen(new WaitCommand(.02))
      .andThen(new LowerArm(_arm)));
    five2.onFalse(new RaiseArm(_arm));
    six2.onTrue(new CloseGripper(_gripper));
    six2.onFalse(new OpenGripper(_gripper));
    
    seven2.onTrue(new LowerGroundControl(_groundControl));
    seven2.onFalse(new RaiseArm(_arm)
      .andThen(new RaiseGroundControl(_groundControl)));
    eight2.onTrue(new RaiseArm(_arm)
      .andThen(new CloseGroundControl(_groundControl)));
    eight2.onFalse(new OpenGroundControl(_groundControl));

    //Right Drive Joystick
    JoystickButton r4 = new JoystickButton(_rightDriveJoystick, 4);
    r4.whileTrue(new AutoPickup(_driveTrain, _arm, _vision, Offset.RIGHT));
    
    JoystickButton r3 = new JoystickButton(_rightDriveJoystick, 3);
    r3.whileTrue(new AutoPickup(_driveTrain, _arm, _vision, Offset.LEFT));

    JoystickButton r5 = new JoystickButton(_rightDriveJoystick, 5);
    r5.whileTrue(new AutoPickup(_driveTrain, _arm, _vision, Offset.CENTER));

    //Left Drive Joystick
    JoystickButton l4 = new JoystickButton(_leftDriveJoystick, 4);
    l4.whileTrue(new LockPosition(_driveTrain));

    JoystickButton l3 = new JoystickButton(_leftDriveJoystick, 3);
    l3.whileTrue(new Balance(_driveTrain));

    JoystickButton r7 = new JoystickButton(_rightDriveJoystick, 7);
    r7.onTrue(new ResetArmEncoder(_arm));    

    // expiremental - remove before competition

    JoystickButton l7 = new JoystickButton(_leftDriveJoystick, 7);
    l7.whileTrue(new BasicBalance(_driveTrain, .3, 1));
    JoystickButton l8 = new JoystickButton(_leftDriveJoystick, 8);
    l8.whileTrue(new Balance(_driveTrain));

    //Throw object?
    JoystickButton l11 = new JoystickButton(_leftDriveJoystick, 11);
    l11.onTrue(new SequentialCommandGroup(
      new LowerGroundControl(_groundControl),
      new WaitCommand(.27),
      new Intake(_groundControl, -1)
    ));

    //For testing turn to angle
    // JoystickButton l9 = new JoystickButton(_leftDriveJoystick, 9);
    // l9.onTrue(new TurnToAngle(_driveTrain, 0, 0));

    // JoystickButton l10 = new JoystickButton(_leftDriveJoystick, 10);
    // l10.onTrue(new TurnToAngle(_driveTrain, 180, 0));

  }

  public void resetPneumatics() {
    if(_functionJoystick2.getRawButton(5)) {
      new OpenGroundControl(_groundControl).schedule();
      new LowerArm(_arm).schedule();
    }
    else {
      new RaiseArm(_arm).schedule();
    }
    if(_functionJoystick2.getRawButton(8)) {
      new RaiseArm(_arm).schedule();
      new CloseGroundControl(_groundControl).schedule();
    }
    else {
      new OpenGroundControl(_groundControl).schedule();
    }
    if(_functionJoystick2.getRawButton(7)) {
      new LowerGroundControl(_groundControl).schedule();
    }
    else {
      new RaiseArm(_arm).schedule();
      new RaiseGroundControl(_groundControl).schedule();
    }
    if(_functionJoystick.getRawButton(6)) {
      new CloseGripper(_gripper).schedule();
    }
    else {
      new OpenGripper(_gripper).schedule();
    }
  }

  public void autoChooserInit() {
    String[] autoselector = {
      "Left", "Center", "Right", "Left score mid", "Center score double", "Right score mid"
    };
    SmartDashboard.putStringArray("Auto List", autoselector);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    String selection = SmartDashboard.getString("Auto Selector", "None");
    Command autoCommand = new StopDrive(_driveTrain); //The default command will be to stop if nothing is selected

    autoCommand = _commands.selectCommand(selection);
    return autoCommand;

    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  }
}
