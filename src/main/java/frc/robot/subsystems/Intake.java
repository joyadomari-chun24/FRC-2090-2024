package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Intake extends SubsystemBase {
    private final TalonSRX intakeController;
    private final double intakeSpeed;
    private final DoubleSolenoid m_doubleSolenoid;
    private final Compressor m_compressor;
    private final DigitalInput breakbeam;
    public final Trigger noteIntaked = new Trigger(this::getBreakbeam);
    public boolean intakeNoLimits = false;

    public Intake() {
        intakeController = new TalonSRX(motorID);
        intakeSpeed = maxSpeed;
        m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
        breakbeam = new DigitalInput(0);
        
        //pressure switch actually turns off the pressurizer at around 125-130 psi ????
        //gauge might be bad, but it works 

        m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        SmartDashboard.putBoolean("Compressor Running", m_compressor.isEnabled());
        SmartDashboard.putBoolean("Intake Full", breakbeam.get());
    }

    public void engageIntake() {
        extendIntake();
        runIntake();
    }

    public void disengageIntake() {
        retractIntake();
        stopIntake();
    }

    public void extendIntake() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntake() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void runIntake() {
        intakeController.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    public void reverseIntake() {
        intakeController.set(ControlMode.PercentOutput, intakeSpeed);
    }

    public void stopIntake() {
        intakeController.set(ControlMode.PercentOutput, 0);   
    }

    public void moveToTransfer() {
        intakeController.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    public void stopMoveToTransfer() {
        intakeController.set(ControlMode.PercentOutput, 0);
    }

    public boolean getBreakbeam() {
        return breakbeam.get();
    }

    public void removeIntakeLimits()  { intakeNoLimits = true;  }

    public void disableCompressor() { m_compressor.disable(); }

    //i dunno if it works so ill just comment it out for now
    // public void toggleIntake() {
    //     m_doubleSolenoid.toggle();
    // }
}
