package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class LEDs extends SubsystemBase {
    
    private static LEDs instance;

    public static LEDs getInstance(){
        if(instance == null){
            instance = new LEDs();
        }
        return instance;
    }
    
    private AddressableLED leds;
    
    private static final int length = 60;
    private static final int port = 0;
        
        

    private LEDs() {
        leds = new AddressableLED(port);
        leds.setLength(length);
        leds.start();
        
    }


//    public Command solid(AddressableLEDBuffer buffer) {
//        return this.runOnce(() -> this.leds.setData(buffer)).repeatedly();
//    }

    public Command solid(AddressableLEDBuffer buffer) {
        return this.runOnce(() -> this.leds.setData(buffer)).;
    }

    public Command flashForever(AddressableLEDBuffer buffer) {
        return sequence(solid(buffer), waitSeconds(0.1), solid(Constants.LEDConstants.OFF), waitSeconds(0.1))
                .repeatedly();
    }
    
    public Command flashOnce(AddressableLEDBuffer buffer) {
        return sequence(solid(buffer), waitSeconds(0.1), solid(Constants.LEDConstants.OFF), waitSeconds(0.1)).;
    }
    
    
    @Override
    public void periodic() {
    }
    
}
