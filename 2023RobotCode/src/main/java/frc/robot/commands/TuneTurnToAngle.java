// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.subsystems.DriveTrain;

public class TuneTurnToAngle extends CommandBase {

  //Set up shuffleboard widgets
  // private ShuffleboardTab tab = Shuffleboard.getTab("Tune");
  // private NetworkTableEntry intakeVoltage = tab.add("Intake Voltage", 0)
  //   .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10.5)).getEntry();
  
  private ShuffleboardTab tab = Shuffleboard.getTab("Tune PID");
  private GenericEntry setPID = tab.add("Set PID Values", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  // private NetworkTableEntry setPID = tab.add("Set PID Values", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  private Boolean setPIDState = false;
  
  // private NetworkTableEntry P = tab.add("P", 0)
  //   .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry P = tab.add("P", 0)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
    // private NetworkTableEntry I = tab.add("I", 0)
    //   .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry I = tab.add("I", 0)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  // private NetworkTableEntry IZone = tab.add("I Zone", 0)
  //   .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 1000)).getEntry();//Max might not be high enough
  private GenericEntry IRange = tab.add("IRange", 0)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 1)).getEntry();//Max might not be high enough
  // private NetworkTableEntry D = tab.add("D", 0)
  //   .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry D = tab.add("D", 0)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();

  // private NetworkTableEntry intakeTargetRPM = tab.add("Target RPM", 0)
  //   .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", -100, "max", 100)).getEntry();//I don't know if the range is correct
  // private GenericEntry TargetAngle = tab.add("Target Angle", 0)
  //   .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", -1080, "max", 1080)).getEntry();//I don't know if the range is correct

  //Get subsystem
  private final DriveTrain _driveTrain;


  /** Creates a new TuneIntake. */
  public TuneTurnToAngle(DriveTrain driveTrain) {
    _driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // new NetworkButton(intakeEnable).whenPressed(this::setPID);
    setPIDState = setPID.getBoolean(false);

    //Grab PID info from shuffleboard and apply it
    setPID();
    Timer.delay(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double targetAngle = TargetAngle.getDouble(0);

    // TurnToAngle(_driveTrain, targetAngle, 5.0);

  

    //Get target RPM from shuffle board
    // double targetRPM = intakeTargetRPM.getDouble(0);

    //Set intake to spin at target RPM
    // _intake.setRPM(targetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setPID() {
    double kP = P.getDouble(0);
    double kI = I.getDouble(0);
    double IRange = this.IRange.getDouble(0);
    double kD = D.getDouble(0);

    // System.out.println(kP + ',' + kI + ',' + kD);
    
    _driveTrain.setPID(kP, kI, IRange, kD);
    

  }
}