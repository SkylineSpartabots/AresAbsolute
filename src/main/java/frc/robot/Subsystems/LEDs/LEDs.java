package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private static LEDs instance;

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    

    public LEDs() {
         leds = new AddressableLED(0);
         ledBuffer = new AddressableLEDBuffer(15); //long strip length is 1000
    }

    public void initLEDs(){
        leds.setLength(ledBuffer.getLength());
    }

    public void updateAlliance(){

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue){
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, 120,255,255);
            }
        } else if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, 0, 255, 255);
            }
        }
        
        leds.setData(ledBuffer);
        leds.start();
        leds.close();
    }

    public void error(){
        /* LEDPattern base = LEDPattern.solid(Color.kOrange);
        LEDPattern pattern = base.blink(Seconds.of(1));

        pattern.applyTo(ledBuffer);
        leds.setData(ledBuffer); */

        for(int i=0; i<ledBuffer.getLength(); i++){
            ledBuffer.setHSV(i, 5, 255, 255);
        }

        leds.setData(ledBuffer);
        leds.start();
    }

}
