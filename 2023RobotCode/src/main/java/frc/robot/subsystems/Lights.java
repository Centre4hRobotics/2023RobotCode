// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private AddressableLED stripLED;
  private AddressableLEDBuffer stripBuffer;

  
  public Lights() {
    stripLED = new AddressableLED(0);
    //rightLED = new AddressableLED(1);
    stripBuffer = new AddressableLEDBuffer(12);
    stripLED.setLength(stripBuffer.getLength());
    //rightBuffer = new AddressableLEDBuffer(6);
    //rightLED.setLength(rightBuffer.getLength());
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setOut(boolean value1, boolean value2) {
   
    if (value1){
      setCube();
    }
    else if(value2){
      setCone();
    }
    else {
      setOff();
    }
    
  }


  public void setOff() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lights").getEntry("state").setValue("off");
    

    for (var i = 0; i < stripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      stripBuffer.setRGB(i, 0,0, 0);//just for testing
   }
   
   stripLED.setData(stripBuffer);
   stripLED.start();
   /*for (var i = 0; i < rightBuffer.getLength(); i++) {
    // Sets the specified LED to the RGB values for red
    rightBuffer.setRGB(i, 0,0, 0);
 }
 
 rightLED.setData(rightBuffer);
 */
}

  public void setCube() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lights").getEntry("state").setValue("cube");
    for (var i = 0; i < stripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      stripBuffer.setRGB(i, 255,0, 255);
   }
   
   stripLED.setData(stripBuffer);
stripLED.start();
   /*for (var i = 0; i < rightBuffer.getLength(); i++) {
    // Sets the specified LED to the RGB values for red
    rightBuffer.setRGB(i, 153,0, 153);
 }
 
 rightLED.setData(rightBuffer);*/


  }
  public void setCone() {
    
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lights").getEntry("state").setValue("cone");
    for (var i = 0; i < stripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      stripBuffer.setRGB(i, 255,255, 0);
   }
   
   stripLED.setData(stripBuffer);
   stripLED.start();
/*   for (var i = 0; i < rightBuffer.getLength(); i++) {
    // Sets the specified LED to the RGB values for red
    rightBuffer.setRGB(i, 255,255, 0);
 }
 
 rightLED.setData(rightBuffer);*/
  }
  
  public void setFloorCube() {
   
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lights").getEntry("state").setValue("floorCube");
  }
}
