// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Balance extends CommandBase {
  /** Creates a new Balance. */

  private ShuffleboardTab tab = Shuffleboard.getTab("Tune Balance PID");
  
  private GenericEntry P;
  private GenericEntry I;
  private GenericEntry IRangeTable;
  private GenericEntry D;
  private GenericEntry baseEntry;

  DriveTrain _driveTrain;

  double kp, ki, kd, IRange, base;
  PIDController _PidController;

  

  public Balance(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _PidController = new PIDController(kp, ki, kd);
    _driveTrain = driveTrain;

    if (tab.getComponents().isEmpty()) {
      P = tab.add("P", _driveTrain.getBalancekP())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      I = tab.add("I", _driveTrain.getBalancekI())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      IRangeTable = tab.add("IRange", _driveTrain.getBalanceIRange())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();//Max might not be high enough
      D = tab.add("D", _driveTrain.getBalancekD())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      baseEntry = tab.add("base", _driveTrain.getBalanceBase())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
    } 

    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPID();

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Balance PID").getEntry("running").setValue(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean useIRange = Math.abs(_driveTrain.getRoll())<IRange;// was pitch in practice
    
    if(useIRange) {
      // changing the behavior of IRange to accumulate integral only while within IRange, instead of using it as a max for the accumulation.
      _PidController.setPID(kp, ki, kd);
    }
    else {
      _PidController.setPID(kp, 0, kd);
    }

    double pidValue = _PidController.calculate(_driveTrain.getRoll(), 0);
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Balance PID").getEntry("RobotPitch").setValue(_driveTrain.getRoll());
    if(pidValue>0 && !useIRange) { //adds a base motor power to overcome friction
      pidValue+=base;
    }
    else if(!useIRange){
      pidValue-=base;
    }
    double maxVal = .65;
    if(pidValue>maxVal) { // caps pidValue to maxVal
      pidValue = maxVal;
    }
    else if(pidValue<-maxVal) {
      pidValue=-maxVal;
    }

    if(Math.abs(_driveTrain.getRoll())>5) {//Was 1
      _driveTrain.arcadeDrive(pidValue, 0);
    } else {
      _driveTrain.arcadeDrive(0, 0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDriveVolts(0, 0);

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Balance PID").getEntry("running").setValue(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setPID() {
    if (P == null) {
      kp = _driveTrain.getBalancekP();
      ki = _driveTrain.getBalancekI();
      IRange = _driveTrain.getBalanceIRange();
      kd = _driveTrain.getBalancekD();
      base = _driveTrain.getBalanceBase();
    } else {
      kp = P.getDouble(_driveTrain.getBalancekP());
      ki = I.getDouble(_driveTrain.getBalancekI());
      IRange = this.IRangeTable.getDouble(_driveTrain.getBalanceIRange());
      kd = D.getDouble(_driveTrain.getBalancekD());
      base = baseEntry.getDouble(_driveTrain.getBalanceBase());
      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      nt.getTable("Balance PID").getEntry("P").setValue(kp);
      nt.getTable("Balance PID").getEntry("I").setValue(ki);
      nt.getTable("Balance PID").getEntry("D").setValue(kd);
      nt.getTable("Balance PID").getEntry("IRange").setValue(IRange);
      nt.getTable("Balance PID").getEntry("base power").setValue(base);
    }
  }

}
