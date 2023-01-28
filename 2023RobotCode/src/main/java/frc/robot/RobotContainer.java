// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToPosition;
import frc.robot.commands.StopDrive;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.UpdateOdometry;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FalconDrive;
import frc.robot.subsystems.NeoDrive;
import frc.robot.subsystems.Vision;

import java.util.function.ToLongBiFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain _driveTrain = new FalconDrive(); // change to neoDrive for a neo bot
  private final Vision _vision = new Vision();
  private final Joystick _leftDriveJoystick = new Joystick(2);//For tank drive
  private final Joystick _rightDriveJoystick = new Joystick(3);//For tank drive

  private final Joystick _functionJoystick = new Joystick(1);

  private final Commands _commands = new Commands(_driveTrain);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _driveTrain.setDefaultCommand(new DriveWithJoysticks(_driveTrain, _leftDriveJoystick, _rightDriveJoystick));// for tank drive
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

    JoystickButton r7 = new JoystickButton(_rightDriveJoystick, 7);
    // r7.whileHeld(new TuneTurnToAngle(_driveTrain));
    r7.onTrue(new Balance(_driveTrain));

    JoystickButton r8 = new JoystickButton(_rightDriveJoystick, 8);
    // r8.whileHeld(new TurnToAngle(_driveTrain, 0, 0));
    r8.onTrue(new TurnToAngle(_driveTrain, 0, 1));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    JoystickButton r9 = new JoystickButton(_rightDriveJoystick, 9);
    r9.onTrue(new TurnToAngle(_driveTrain, new Pose2d(45, 45, new Rotation2d(0)), 1));

    JoystickButton r10 = new JoystickButton(_rightDriveJoystick, 10);
    r10.onTrue(new GoToPosition(_driveTrain, new Pose2d(14, 3.88, new Rotation2d(0)), _vision));

    JoystickButton r11 = new JoystickButton(_rightDriveJoystick, 11);
    r11.onTrue(new UpdateOdometry(_vision, _driveTrain));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public void autoChooserInit() {
    String[] autoselector = {"1-ball(LOW) Fender", "test"};
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
