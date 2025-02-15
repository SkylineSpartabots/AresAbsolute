package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.Seconds;
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
         ledBuffer = new AddressableLEDBuffer(1000); //long strip length is 1000
    }

    public void initLEDs(){
        leds.setLength(ledBuffer.getLength());
    }

    public enum ledStates{
        ORANGE(5, 255, 255),
        BLUE(120, 255, 255),
        RED(0, 255, 255);

        public  int h;
        public int s;
        public int v;
        
        public int getH(){
            return h;
        }
        public int getS(){
            return s;
        }
        public int getV(){
            return v;
        }

        ledStates(int h, int s, int v){
            this.h = h;
            this.s = s;
            this.v = v;
        }
    }

    public void setColor(ledStates state){
        for(int i=0; i<ledBuffer.getLength(); i++){
            ledBuffer.setHSV(i, state.getH(), state.getS(), state.getV());
        }

        leds.setData(ledBuffer);
        leds.start();
    }
}
