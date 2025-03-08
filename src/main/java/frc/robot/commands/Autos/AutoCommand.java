// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
        S1(new AutoCommand[]{S1R1(), S1R2()}), //Source areas
        S2(new AutoCommand[]{S2R9(), S2R10()}),
        R1(new AutoCommand[]{}), //Reef scoring areas
        R2(new AutoCommand[]{}),
        R3(new AutoCommand[]{R3S1(), Wait()}),
        R4(new AutoCommand[]{}),
        R5(new AutoCommand[]{}),
        R6(new AutoCommand[]{}),
        R7(new AutoCommand[]{}),
        R8(new AutoCommand[]{R8S2(), Wait()}),
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

    public static AutoCommand S1R1(){
        return new AutoCommand("S1R1", Autos.S1R1(), EndPoint.R1);
    }

    public Command getCommand(){
        return this.autoCommand;
    }

    public static AutoCommand S1R2(){
        return new AutoCommand("S1R2", Autos.S1R2(), EndPoint.R2);
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

    public static AutoCommand Wait(){
        return new AutoCommand("wait", Commands.waitSeconds(10), EndPoint.GENERIC);
    }

    public static AutoCommand B1R3(){
        return new AutoCommand("B1R3", Autos.B1R3(), EndPoint.R3);
    }

    public static AutoCommand B2R8(){
        return new AutoCommand("B2R8", Autos.B2R8(), EndPoint.R8);
    }

    public static AutoCommand R8S2(){
        return new AutoCommand("R8S2", Autos.R8S2(), EndPoint.S2);
    }

    public static AutoCommand R3S1(){
        return new AutoCommand("R3S1", Autos.R3S1(), EndPoint.S1);
    }

    public static AutoCommand S2R10(){
        return new AutoCommand("S2R10", Autos.S2R10(), EndPoint.R10);
    }

    public static AutoCommand R10S2(){
        return new AutoCommand("R10S2", Autos.R10S2(), EndPoint.S2);
    }

    public static AutoCommand S2R9(){
        return new AutoCommand("S2R9", Autos.S2R9(), EndPoint.R9);
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
