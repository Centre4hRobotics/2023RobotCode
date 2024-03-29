// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private AddressableLED stripLED;
  private AddressableLEDBuffer stripBuffer;
  private int lightActive = 3;
  private int count = 0;
  private boolean desc = false;
  private boolean wasOn = true;

  
  public Lights() {
    stripLED = new AddressableLED(0);
    //rightLED = new AddressableLED(1);
    stripBuffer = new AddressableLEDBuffer(60);
    stripLED.setLength(stripBuffer.getLength());
    //rightBuffer = new AddressableLEDBuffer(6);
    //rightLED.setLength(rightBuffer.getLength());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    wasOn = true;
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
    wasOn = true;
    
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
  public void pulse() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lights").getEntry("state").setValue("pulse");
    int ticks = 5;
    if(wasOn) {
      for (var i = 0; i < stripBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if(lightActive/ticks==i||lightActive/ticks==i+1) {
          stripBuffer.setRGB(i, 0, 255, 0);
        }
        else {
          stripBuffer.setRGB(i, 0, 0, 0);
        }
      }
      wasOn = false;
    }
    if(lightActive/ticks>0&&lightActive/ticks<stripBuffer.getLength()) {
      if(lightActive/ticks>1) {
        stripBuffer.setRGB(lightActive/ticks-2, 0, 0, 0);
      }
      stripBuffer.setRGB(lightActive/ticks-1, 0, 255, 0);
      stripBuffer.setRGB(lightActive/ticks, 0, 255, 0);
      if(lightActive/ticks<stripBuffer.getLength()-1) {
        stripBuffer.setRGB(lightActive/ticks+1, 0, 0, 0);
      }
    }
    if(desc) {
      lightActive-=1;
    }
    else {
      lightActive+=1;
    }
    if(lightActive==(stripBuffer.getLength()+1)*ticks-1||lightActive==1) {
      desc = !desc;
    }
    stripLED.setData(stripBuffer);
    stripLED.start();
  }
  public void pulse2() {
    Color backround = new Color(0, 0, 0);
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lights").getEntry("state").setValue("pulse");
    int ticks = 5;
    if(wasOn) {
      for (var i = 0; i < stripBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if((lightActive/ticks)%(stripBuffer.getLength()/3)==i||(lightActive/ticks)%(stripBuffer.getLength()/3)==i+1) {
          stripBuffer.setRGB(i, 0, 255, 0);
        }
        else {
          stripBuffer.setRGB(i, 0, 0, 0);
        }
      }
      wasOn = false;
    }
    if(lightActive/ticks>0&&lightActive/ticks<stripBuffer.getLength()) {
      if(lightActive/ticks>1) {
        stripBuffer.setLED(lightActive/ticks-2, backround);
        stripBuffer.setLED(lightActive/ticks-2+(stripBuffer.getLength()/3), backround);
        stripBuffer.setLED(lightActive/ticks-2+(stripBuffer.getLength()/3)*2, backround);
      }
      stripBuffer.setRGB(lightActive/ticks-1, 0, 255, 0);
      stripBuffer.setRGB(lightActive/ticks, 0, 255, 0);
      stripBuffer.setRGB(lightActive/ticks-1+(stripBuffer.getLength()/3), 0, 255, 0);
      stripBuffer.setRGB(lightActive/ticks+(stripBuffer.getLength()/3), 0, 255, 0);
      stripBuffer.setRGB(lightActive/ticks-1+(stripBuffer.getLength()/3)*2, 0, 255, 0);
      stripBuffer.setRGB(lightActive/ticks+(stripBuffer.getLength()/3)*2, 0, 255, 0);
      if(lightActive/ticks<stripBuffer.getLength()/3-1) {
        stripBuffer.setLED(lightActive/ticks+1, backround);
        stripBuffer.setLED(lightActive/ticks+1+(stripBuffer.getLength()/3), backround);
        stripBuffer.setLED(lightActive/ticks+1+(stripBuffer.getLength()/3)*2, backround);
      }
    }
    if(desc) {
      lightActive-=1;
    }
    else {
      lightActive+=1;
    }
    if(lightActive==(stripBuffer.getLength()/3)*ticks-1||lightActive==1) {
      desc = !desc;
    }
    stripLED.setData(stripBuffer);
    stripLED.start();
  }
  
  public void setFloorCube() {
    wasOn = true;
   
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lights").getEntry("state").setValue("floorCube");
  }

  public void scroll(int max) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lights").getEntry("state").setValue("scroll");

    if (lightActive == 0) {
      final int n = 3;

      for (int i = 0; i < max; i++) {
        int j = i + count;
        if (j >= max) j -= max;

        // get scaled brightness based on position
        // int val = (int) (255 * ((double) j / max));
        int val = (int) (20.0 * (double) j / max + 128 * (max - j));
        // alternate colors every n
        if ((i / n) % 2 == 0) {
          stripBuffer.setRGB(j, 0, val, 0);
        } else {
          stripBuffer.setRGB(j, val, val, val);
        }
      }
      
      count++;
      if (count >= max) count = 0;
      System.out.println(count);

      stripLED.setData(stripBuffer);
      stripLED.start();
    }
    lightActive++;
    if (lightActive >= 4) lightActive = 0;
  }

  public void scroll() {
    scroll(stripBuffer.getLength());
  }
}
