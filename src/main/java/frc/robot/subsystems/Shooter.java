package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
    private final TalonSRX shooterControllerL;
    private final TalonSRX shooterControllerR;
    private double velocityL;
    private double velocityR;

    //Set velocity (look into this): https://pro.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/closed-loop-guide.html
    public final Trigger shooterIsSpunUp = new Trigger(this::shooterSpunUp);
    Slot0Configs slot0PIDConfigs = new Slot0Configs();

    public Shooter() {
        shooterControllerL = new TalonSRX(leftMotorID); 
        shooterControllerR = new TalonSRX(rightMotorID); 
        slot0PIDConfigs.kS = kS;
        slot0PIDConfigs.kV = kV;
        slot0PIDConfigs.kA = kA;
        slot0PIDConfigs.kP = kP;
        slot0PIDConfigs.kI = kI;
        slot0PIDConfigs.kD = kD;
        //shooterControllerL.getConfigurator().apply(slot0PIDConfigs);
    }

    @Override
    public void periodic() {
        //getSelectedSensorVelocity returns ticks per 100ms, so we can just convert to rpm w/ dimensional analysis
        velocityL = shooterControllerL.getSelectedSensorVelocity() / 4096 * 1000 * 60;
        velocityR = shooterControllerR.getSelectedSensorVelocity() / 4096 * 1000 * 60;
        SmartDashboard.putNumber("Shooter Speed", maxSpeed);
        SmartDashboard.putNumber("Shooter Velocity (Left)", velocityL);
        SmartDashboard.putNumber("Shooter Velocity (Right)", velocityR);
    }


    public void runShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, -maxSpeed);
        shooterControllerR.set(ControlMode.PercentOutput, maxSpeed);
    }

    public void runShooterEject() {
        shooterControllerL.set(ControlMode.PercentOutput, -ejectSpeed);
        shooterControllerR.set(ControlMode.PercentOutput, ejectSpeed);
    }

    public void stopShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, 0);
        shooterControllerR.set(ControlMode.PercentOutput, 0);
    }

    public void reverseShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, maxSpeed);
        shooterControllerR.set(ControlMode.PercentOutput, -maxSpeed);
    }

    // checks if shooter is ready
    public boolean shooterSpunUp() {
        if (velocityL >= spunUpVelocity)
            return true;

        return false;
    }

    public double getVelocity() {
        return velocityL;
    }

    // public double adjustVelocity(double oldSpeed, DoubleSupplier adjuster)
    // {
    //     adjuster = MathUtil.applyDeadband(adjuster.getAsDouble(), adjustmentDeadband);
    //     return oldSpeed - adjuster; //my head hurts. I'll do this later.
    // }
}

