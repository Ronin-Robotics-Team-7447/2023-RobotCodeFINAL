package com.team6560.frc2023.subsystems;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledbuffer;
    private AddressableLED led1;
    private AddressableLEDBuffer ledbuffer1;
    String mode;

    public Lights() {
        led = new AddressableLED(0);
        // led1 = new AddressableLED(2);
        ledbuffer = new AddressableLEDBuffer(17);
        // ledbuffer1 = new AddressableLEDBuffer(11);
        // led1.setLength(ledbuffer1.getLength());
        led.setLength(ledbuffer.getLength());
        setLightsToDefault();
    }

    public void setLightsToDefault() {
        //
        
        rainbow();
        mode = "default";
        ledbuffer.setRGB(0, 41, 202, 254 );
        ledbuffer.setRGB(1, 255, 255, 255 );
        ledbuffer.setRGB(2, 41, 202, 254 );
        ledbuffer.setRGB(3, 255, 255, 255 );
        ledbuffer.setRGB(4, 41, 202, 254 );
        ledbuffer.setRGB(5, 255, 255, 255 );
        led.setData(ledbuffer);
        led.start();


        // ledbuffer1.setRGB(0, 41, 202, 254 );
        // ledbuffer1.setRGB(1, 255, 255, 255 );
        // ledbuffer1.setRGB(2, 41, 202, 254 );
        // ledbuffer1.setRGB(3, 255, 255, 255 );
        // ledbuffer1.setRGB(4, 41, 202, 254 );
        // ledbuffer1.setRGB(5, 255, 255, 255 );
        // ledbuffer1.setRGB(6, 41, 202, 254 );
        // ledbuffer1.setRGB(7, 255, 255, 255 );
        // ledbuffer1.setRGB(8, 41, 202, 254 );
        // ledbuffer1.setRGB(9, 255, 255, 255 );
        // ledbuffer1.setRGB(10, 41, 202, 254 );
        // led1.setData(ledbuffer1);
        // led1.start();
    }

    public void setLightsToCone() {
        mode = "cone";
        for( int i = 0; i < ledbuffer.getLength(); i++ ) {
            ledbuffer.setRGB(i, 255, 255, 0);
        }
        led.setData(ledbuffer);
        led.start();
    }

    public void setLightsToCube() {
        mode = "cube";
        for( int i = 0; i < ledbuffer.getLength(); i++ ) {
            ledbuffer.setRGB(i, 128, 0, 128);
        }
        led.setData(ledbuffer);
        led.start();
    }

    public String getMode() {
        return mode;
    }

    public void setLights(int red, int green, int blue) {
        for( int i = 0; i < ledbuffer.getLength(); i++ ) {
            ledbuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledbuffer);
        led.start();
    }

    public void rainbow() {
        int m_rainFirstPixelHue = 1;
        for( int i = 6; i <= ledbuffer.getLength()-1; i++ ) {
            int hue = (m_rainFirstPixelHue + ( i * 180 / ledbuffer.getLength()-6)) % 180;
            ledbuffer.setHSV(i, hue, 255, 128);
            m_rainFirstPixelHue += 2;
            m_rainFirstPixelHue %= 180;
        }
    }
    // @Override
    // public void periodic() {
    //     int setEmpty = 6;
    //     for( int i = 6; i < ledbuffer.getLength()-1; i++ ) {
            
    //     }
    //     ledbuffer.setRGB(setEmpty, 41, 202, 254);
    //     led.start();

    //     if( setEmpty == 16 ) {
    //         setEmpty 
    //     }
    // }
}
