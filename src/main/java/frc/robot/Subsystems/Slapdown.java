package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// What am I doing :'D
public class Slapdown extends SubsystemBase {
    private TalonFX pivotM;
    private TalonFX rollerM;

    public Slapdown() {
        pivotM = new TalonFX(Constants.HardwarePorts.slapdownPivot); // Remember to change Constant number
        rollerM = new TalonFX(Constants.HardwarePorts.slapdownIntake); // This one too
    }

    // TODO: Config later?

    public enum PivotState {
        UP(10), // Placeholder nums...
        DOWN(5);

        private double position;

        private PivotState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum RollerState {
        INTAKE(0.5),
        OUTTAKE(-0.5),
        OFF(0);

        private double motorSpeed;

        private RollerState(double motorSpeed) {
            this.motorSpeed = motorSpeed;
        }

        public double getRollerSpeed() {
            return motorSpeed;
        }
    }

    public void stopPivot() {
        pivotM.set(0);
    }

    public double getPosition() {
        return pivotM.getPosition().getValueAsDouble();
    }

    public void setPivotSpeed(double speed) {
        pivotM.set(speed);
    }

    public void setPivotVoltage(double voltage) {
        pivotM.setControl(new VoltageOut(voltage));
    }

    public void setRollerSpeed(double speed) {
        rollerM.set(speed);
    }

    public double getRollerCurrent() {
        return rollerM.getStatorCurrent().getValueAsDouble();
    }

    public void setRollerVoltage(double voltage) {
        rollerM.setControl(new VoltageOut(voltage));
    }

    public void setIntakeSpeed(double speed) {
        rollerM.set(speed);
    }
}
