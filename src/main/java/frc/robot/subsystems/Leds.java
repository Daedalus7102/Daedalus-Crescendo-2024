// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;

import java.util.Random;


public class Leds extends SubsystemBase {
  
  public Leds() {m_led.setLength(m_ledBuffer.getLength()); m_led.start(); timer.start();}

  // -- CONFIG -- //
  private static final int LENGTH = 25; // La mitad de la cantidad de leds que hay
  private static final int PORT = 0; // El puerto PWM en el que se conectan los leds

  private AddressableLED m_led = new AddressableLED(PORT);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LENGTH*2);
  private Color[] drawBuffer = new Color[LENGTH];
  private int state = 0;
  private Timer timer = new Timer();
  private Random random = new Random();

  public void setEffect(int effectID) {
    state = effectID;
  }

  // -- EFECTOS -- //
  private void draw() {

    if (state == 0) {
      blowtorch();
    } else if (state == 1) {
      pulseGreen();
    } else if (state == 2) {
      bounce();
    }

    for (int i = 0; i < LENGTH; i++) {
      drawBuffer[i] = m_ledBuffer.getLED(i);
    }

    for (int i = LENGTH; i < LENGTH*2; i++) {
      m_ledBuffer.setLED(i, drawBuffer[i-LENGTH]);
    }

  }

  private int[] heat = new int[LENGTH];
  private static final int COOLING = 100;
  private static final boolean REVERSE_DIRECTION = false;
  private void blowtorch() {
    int buf = random.nextInt(250, 280);
    for (int i = 0; i <= random.nextInt(2, 4); i++){
      heat[i] = buf;
      //heat[i] = random.nextInt(200, 255);
    }

    for( int i = 0; i < LENGTH; i++) {
      heat[i] = heat[i] - random.nextInt(0, ((COOLING * 10) / LENGTH) + 5);
      heat[i] = heat[i] < 0 ? 0 : heat[i];
    }
  
    for( int k= LENGTH - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
  
    for( int j = 0; j < LENGTH; j++) {
      Color color = ColorUtils.heatColor( heat[j]);
      int pixelnumber;
      if ( REVERSE_DIRECTION ) {
        pixelnumber = (LENGTH-1) - j;
      } else {
        pixelnumber = j;
      }
      m_ledBuffer.setLED(pixelnumber, color);
    }
  }

  private void pulseGreen() {
    double pos = Math.sin(timer.get() * 20);
    if (pos >= 0) {
      for (int i=0; i < LENGTH; i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      }
    } else {
      for (int i=0; i < LENGTH; i++) {
        m_ledBuffer.setLED(i, Color.kBlack);
      }
    }
  }

  private void bounce() {
    float calc = LENGTH-1;

    int pos = (int) Math.round(Math.sin(timer.get() * 4) * calc/2 + calc/2);
    for (int i =0; i < LENGTH; i++){
      m_ledBuffer.setLED(i, Color.kBlack);
    }

    if (pos > LENGTH - 2) {
      m_ledBuffer.setHSV(pos, 30, 255, 150);
    }
    if (pos > LENGTH - 1) {
      m_ledBuffer.setHSV(pos-1, 30, 255, 200);
    }
    m_ledBuffer.setHSV(pos, 30, 255, 255);
    if (pos < LENGTH + 1) {
      m_ledBuffer.setHSV(pos, 30, 255, 200);
    }
    if (pos < LENGTH + 2) {
      m_ledBuffer.setHSV(pos, 30, 255, 150);
    }
  } 

  

  private static class ColorUtils {
    private static Color heatColor(int temperature) {
      // Pura magia negra
      int t191 = scale32Video(temperature, 191);
  
      int heatramp = t191 & 0x3F; // 0..63
      heatramp <<= 2; // scale up to 0..252
      
      int[] buf = new int[3];
      if ((t191 & 0x80) != 0) {
          buf[0] = 255;
          buf[1] = 255;
          buf[2] = heatramp;
      } else if ((t191 & 0x40) != 0) {
          buf[0] = 255;
          buf[1] = heatramp;
          buf[2] = 0;
      } else {
          buf[0] = heatramp;
          buf[1] = 0;
          buf[2] = 0;
      }
      Color heatcolor = new Color(buf[0], buf[1], buf[2]);

      return heatcolor;
    }

    private static int scale32Video(int i, int scale) {
    // Pura magia negra
      int j;
   
      if (i != 0) {
          long result = (long) i * (long) scale;
          j = (int) (result / 255); // Divide by 255 to simulate the behavior of 8-bit integers
          if (scale != j) {
              j -= 255;
          }
      } else {
          j = 0;
      }
  
      return j;
    }    
  }

  @Override
  public void periodic() {
    state = 2;
    draw();
    m_led.setData(m_ledBuffer);
  }
}
