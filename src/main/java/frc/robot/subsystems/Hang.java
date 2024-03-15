package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.Hang.*;

public class Hang extends SubsystemBase {
    private final CANSparkMax leftHangController;
    private final CANSparkMax rightHangController;
    private final double hangSpeed;
    public double leftEncoder = 0;
    public double rightEncoder = 0;
    private double upperLimit = -49; 
    private double lowerLimit = 0;
    private boolean limits = true;

    public Hang() {
        leftHangController = new CANSparkMax(leftHangID, MotorType.kBrushless); 
        rightHangController = new CANSparkMax(rightHangID, MotorType.kBrushless); 
        hangSpeed = maxSpeed;
    }

    @Override
    public void periodic() {
        leftEncoder =  leftHangController.getEncoder().getPosition();
        rightEncoder = rightHangController.getEncoder().getPosition();
        SmartDashboard.putNumber("Left Hang Current", leftHangController.getOutputCurrent());
        SmartDashboard.putNumber("Right Hang Current", rightHangController.getOutputCurrent());
        SmartDashboard.putNumber("Left Encoder", leftEncoder);
        SmartDashboard.putNumber("Right Encoder", rightEncoder);
    }

    public void runHang(int input) {    
        /*
         * If the hang is beyond its limit, it can only go in a favorable direction (e.g it can 
         * only go down if it's too high).
         */

        if (!limits) {
            return;
        }
        
        boolean atMaxHeight = false;
        boolean atMinHeight = false;

        if (leftEncoder > lowerLimit || rightEncoder > lowerLimit) {
            atMinHeight = true;
        }
        else if (leftEncoder < upperLimit || rightEncoder < upperLimit) {
            atMaxHeight = true;
        }

        if (atMinHeight && input < 0) {
            leftHangController.set(maxSpeed * input);
            rightHangController.set(maxSpeed * input);
        }
        else if (atMaxHeight && input > 0) {
            leftHangController.set(maxSpeed * input);
            rightHangController.set(maxSpeed * input);
        }
        else {
            leftHangController.set(maxSpeed * input);
            rightHangController.set(maxSpeed * input);
        }
    }

    public void stopHang() {
        leftHangController.set(0);
        rightHangController.set(0);
    }

    public void removeHangLimits() { limits = false; }

    public Command runHangUp() {
        return this.runEnd(() -> this.runHang(-1), () -> this.stopHang());
    }

    public Command runHangDown() {
        return this.runEnd(() -> this.runHang(1), () -> this.stopHang());
    }
}

