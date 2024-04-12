// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Random;

public class Leds extends SubsystemBase {
  
  public Leds() {m_led.setLength(m_ledBuffer.getLength()); m_led.start();}

  // -- CONFIG -- //
  private static final int LENGTH = 25; // La mitad de la cantidad de leds que hay
  private static final int PORT = 0; // El puerto PWM en el que se conectan los leds

  private AddressableLED m_led = new AddressableLED(PORT);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LENGTH*2);
  private Random random = new Random();

  // -- EFECTOS -- //
  private void draw(ColorUtils.RGB[] colors) {
    for (int i = 0; i < LENGTH; i++) {
      m_ledBuffer.setRGB(i, colors[i].r, colors[i].g, colors[i].b);
    }
    for (int i = LENGTH; i < LENGTH*2; i++) {
      m_ledBuffer.setRGB(i, colors[i-LENGTH].r, colors[i-LENGTH].g, colors[i-LENGTH].b);
    }

  }

  private int[] heat = new int[LENGTH];
  private static final int COOLING = 100;
  private static final boolean REVERSE_DIRECTION = false;
  private ColorUtils.RGB[] blowtorch() {
    ColorUtils.RGB[] colorBuffer = new ColorUtils.RGB[LENGTH];
    int buf = random.nextInt(250, 280);
    for (int i = 0; i <= random.nextInt(2, 4); i++){
      heat[i] = buf;
      //heat[i] = random.nextInt(200, 255);
    }

    for( int i = 0; i < LENGTH; i++) {
      heat[i] = heat[i] - random.nextInt(0, ((COOLING * 10) / LENGTH) + 5); // qsub saturates at zero, this doesn't!
      heat[i] = heat[i] < 0 ? 0 : heat[i];
    }
  
    for( int k= LENGTH - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
  
    for( int j = 0; j < LENGTH; j++) {
      ColorUtils.RGB color = ColorUtils.heatColor( heat[j]);
      int pixelnumber;
      if ( REVERSE_DIRECTION ) {
        pixelnumber = (LENGTH-1) - j;
      } else {
        pixelnumber = j;
      }
      colorBuffer[pixelnumber] = color;
    }
    return colorBuffer;
  }

  private static class ColorUtils {
    private static RGB heatColor(int temperature) {
      // Pura magia negra
      int t191 = scale32Video(temperature, 191);
  
      int heatramp = t191 & 0x3F; // 0..63
      heatramp <<= 2; // scale up to 0..252
  
      RGB heatcolor = new RGB();
  
      if ((t191 & 0x80) != 0) {
          heatcolor.r = 255;
          heatcolor.g = 255;
          heatcolor.b = heatramp;
      } else if ((t191 & 0x40) != 0) {
          heatcolor.r = 255;
          heatcolor.g = heatramp;
          heatcolor.b = 0;
      } else {
          heatcolor.r = heatramp;
          heatcolor.g = 0;
          heatcolor.b = 0;
      }
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
    private static class RGB {
      public int r;
      public int g;
      public int b;
      }
    }


  @Override
  public void periodic() {
    draw(blowtorch());
    m_led.setData(m_ledBuffer);
  }
}
