// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.CloseGroundControl;
import frc.robot.commands.ControlLights;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ExtendArmWithButtons;
import frc.robot.commands.ExtendArmWithJoystick;
import frc.robot.commands.FollowTrajectoryToPose;
import frc.robot.commands.GetOnChargingStation;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeWithSwitch;
import frc.robot.commands.LowerArm;
import frc.robot.commands.LowerGroundControl;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.OpenGroundControl;
import frc.robot.commands.RaiseArm;
import frc.robot.commands.RaiseGroundControl;
import frc.robot.commands.LockPosition;
import frc.robot.commands.SetArmHeight;
import frc.robot.commands.StopDrive;
import frc.robot.commands.TurnSlow;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.UpdateOdometry;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconDrive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.GroundControl;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.NeoDrive;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final DriveTrain _driveTrain = new FalconDrive(_arm); // change to neoDrive for a neo bot
  private final Gripper _gripper = new Gripper();
  private final Lights _lights = new Lights();
  //private final Vision _vision = new Vision();
  private final Joystick _leftDriveJoystick = new Joystick(2);//For tank drive
  private final Joystick _rightDriveJoystick = new Joystick(3);//For tank drive

  private final Joystick _functionJoystick = new Joystick(1);

  private final Commands _commands = new Commands(_driveTrain, _groundControl, _arm, _gripper);
  


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _arm.get(_groundControl);
    _driveTrain.setDefaultCommand(new DriveWithJoysticks(_driveTrain, _leftDriveJoystick, _rightDriveJoystick));// for tank drive
    _arm.setDefaultCommand(new ExtendArmWithJoystick(_arm, _functionJoystick));
    _groundControl.setDefaultCommand(new IntakeWithSwitch(_groundControl, _functionJoystick, .4));
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
    JoystickButton three = new JoystickButton(_functionJoystick, 3); 
    JoystickButton four = new JoystickButton(_functionJoystick, 4);
    JoystickButton five = new JoystickButton(_functionJoystick, 5); // used by Intake with Switch
    JoystickButton six = new JoystickButton(_functionJoystick, 6); 
    JoystickButton seven = new JoystickButton(_functionJoystick, 7);

    JoystickButton one2 = new JoystickButton(_functionJoystick, 1);
    JoystickButton two2 = new JoystickButton(_functionJoystick, 2);
    JoystickButton three2 = new JoystickButton(_functionJoystick, 3);
    JoystickButton four2 = new JoystickButton(_functionJoystick, 4);
    JoystickButton five2 = new JoystickButton(_functionJoystick, 5);
    JoystickButton six2 = new JoystickButton(_functionJoystick, 6);

    //Function Button Board
    one.onTrue(new RaiseGroundControl(_groundControl));
    one.onFalse(new LowerGroundControl(_groundControl));
    
    two.onTrue(new OpenGroundControl(_groundControl));
    two.onFalse(new CloseGroundControl(_groundControl));

    three.onTrue(new Intake(_groundControl, -.4));
    four.whileTrue(new Intake(_groundControl, .4));

    one2.onTrue(new RaiseArm(_arm));
    one2.onFalse(new LowerArm(_arm));

    five2.onTrue(new SetArmHeight(_arm, ArmConstants.highPosition));
    six2.onTrue(new SetArmHeight(_arm, ArmConstants.middlePosition));
    four2.onTrue(new SetArmHeight(_arm, ArmConstants.pickupPosition));
    three2.onTrue(new SetArmHeight(_arm, ArmConstants.retracted));
    two2.onTrue(new CloseGripper(_gripper));
    two2.onFalse(new OpenGripper(_gripper));
    


    //Right Drive Joystick
    JoystickButton r4 = new JoystickButton(_rightDriveJoystick, 4);
    r4.onTrue(new TurnSlow(_driveTrain, true));
    
    JoystickButton r3 = new JoystickButton(_rightDriveJoystick, 3);
    r3.onTrue(new TurnSlow(_driveTrain, false));



    //Left Drive Joystick
    JoystickButton l4 = new JoystickButton(_leftDriveJoystick, 5);
    l4.whileTrue(new LockPosition(_driveTrain));

    JoystickButton l3 = new JoystickButton(_leftDriveJoystick, 3);
    l3.whileTrue(new Balance(_driveTrain));



    JoystickButton r7 = new JoystickButton(_rightDriveJoystick, 7);
    r7.onTrue(new GetOnChargingStation(_driveTrain, .5, 1).andThen(new LockPosition(_driveTrain)));

    //Comment out Button Bindings below here for competitions (they are tests)
    /*
    JoystickButton r7 = new JoystickButton(_rightDriveJoystick, 7);
    // r7.whileHeld(new TuneTurnToAngle(_driveTrain));
    r7.onTrue(new Balance(_driveTrain));

    // JoystickButton r8 = new JoystickButton(_rightDriveJoystick, 8);
    // r8.onTrue(new TurnToAngle(_driveTrain, 0, 1));

    // JoystickButton r9 = new JoystickButton(_rightDriveJoystick, 9);
    // r9.onTrue(new TurnToAngle(_driveTrain, new Pose2d(45, 45, new Rotation2d(0)), 1));

    JoystickButton r8 = new JoystickButton(_rightDriveJoystick, 8);
    r8.whileTrue(new LockPosition(_driveTrain));

    JoystickButton r10 = new JoystickButton(_rightDriveJoystick, 10);
    // r10.onTrue(new GoToPosition(_driveTrain, new Pose2d(14, 3.88, new Rotation2d(1, 0)), _vision));
    r10.onTrue(new UpdateOdometry(_vision, _driveTrain, true) //Only update pose if it seems like a good pose
      .andThen(new FollowTrajectoryToPose(_driveTrain, new Pose2d(14.5, 4.5, new Rotation2d(1, 0)), .4))
      // .andThen(new ExampleCommand(_driveTrain, new Pose2d(14, 3.88, new Rotation2d(1, 0))))
    );

    JoystickButton r11 = new JoystickButton(_rightDriveJoystick, 11);
    r11.onTrue(new UpdateOdometry(_vision, _driveTrain, false));  //Do a total overwrite

    // JoystickButton r12 = new JoystickButton(_rightDriveJoystick, 12);
    // r12.onTrue(new FollowTrajectoryToPose(_driveTrain, new Pose2d(13.5, 4.5, new Rotation2d(1, 0))));

    JoystickButton l12 = new JoystickButton(_leftDriveJoystick, 12);
    l12.onTrue(new TurnToAngle(_driveTrain, 0, 3));
    */


  }

  public void autoChooserInit() {
    String[] autoselector = {
      "Blue Left", "Blue Center", "Blue Right", "Red Left", "Red Center", "Red Right"
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
