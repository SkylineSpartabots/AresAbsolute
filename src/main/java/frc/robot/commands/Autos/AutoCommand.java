// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.HashMap;

/** Add your docs here. */
public class AutoCommand {
    Command autoCommand;
    public String name;
    private EndPoint end;
    // private static HashMap<String, AutoCommand> pathKeys = new HashMap<String, AutoCommand>();
    private static ArrayList<AutoCommand> autoList2 = new ArrayList<AutoCommand>();

    //if you have questions ask ethan

    public AutoCommand(String name, Command autoCommand, EndPoint endPoint){
        this.name = name;
        this.autoCommand = autoCommand;
        this.end = endPoint;
    }

    private enum EndPoint {
        GENERIC(new AutoCommand[]{}), //use for wait commands, etc. Any paths that can lead into any other path
        S1(new AutoCommand[]{}), //Source areas
        S2(new AutoCommand[]{}),
        R1(new AutoCommand[]{}), //Reef scoring areas
        R2(new AutoCommand[]{}),
        R3(new AutoCommand[]{}),
        R4(new AutoCommand[]{}),
        R5(new AutoCommand[]{}),
        R6(new AutoCommand[]{}),
        R7(new AutoCommand[]{}),
        R8(new AutoCommand[]{}),
        R9(new AutoCommand[]{}),
        R10(new AutoCommand[]{}),
        R11(new AutoCommand[]{}),
        R12(new AutoCommand[]{}),
        C1(new AutoCommand[]{}),
        C2(new AutoCommand[]{}),
        C3(new AutoCommand[]{}),
        C4(new AutoCommand[]{}),
        C5(new AutoCommand[]{}),
        C6(new AutoCommand[]{});
        private AutoCommand[] branches; //this stores all of the paths that start at this endpoint
        private EndPoint(AutoCommand[] branches){
            this.branches = branches;
        }
    }

    public Command getCommand(){
        return this.autoCommand;
    }

    public static void clearContinuations(){
        autoList2.clear();
    }

    public static void fillAutosList(AutoCommand auto){
        for(AutoCommand command : auto.end.branches){
            autoList2.add(command);
        }
    }


    public static ArrayList<AutoCommand> getPotentialContinuations() {
        return autoList2;
    }


    public static AutoCommand Test1(){
        return new AutoCommand("Test1", Autos.Test1(), EndPoint.GENERIC);
    }

    public static AutoCommand Test2(){
        return new AutoCommand("Test2", Autos.Test2(), EndPoint.GENERIC);
    }

    public static AutoCommand Test3() {
        return new AutoCommand("Test3", Autos.Test3(), EndPoint.GENERIC);
    }

    public static AutoCommand Test4() {
        return new AutoCommand("Test4", Autos.Test4(), EndPoint.GENERIC);
    }

    public static AutoCommand meter1() {
        return new AutoCommand("1meter", Autos.meter(), EndPoint.GENERIC);
    }

    public static AutoCommand meter2() {
        return new AutoCommand("2meter", Autos.meter2(), EndPoint.GENERIC);
    }

    public static AutoCommand meter3() {
        return new AutoCommand("3meter", Autos.meter3(), EndPoint.GENERIC);
    }

    public static AutoCommand backandforth() {
        return new AutoCommand("backandforth", Autos.backandforth(), EndPoint.GENERIC);
    }


    // public static AutoCommand meterForwardTest() {
    //     return new AutoCommand("meterForwardTest", Autos.meterForwardTest(), EndPoint.GENERIC);
    // }

    // public static AutoCommand getAuto(String name){
    //     return pathKeys.get(name);
    // }
}
