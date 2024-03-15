package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final DriveControls controls = new DriveControls();

    // /* Drive Controls */
    // private final int translationAxis = Joystick.Axis.kLeftY.value;
    // private final int strafeAxis = Joystick.Axis.kLeftX.value;
    // private final int rotationAxis = Joystick.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Hang s_Hang = new Hang();
    private final GamerLights s_Blinkin = new GamerLights();
    private final Transfer s_Transfer = new Transfer();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> controls.getForward() / 1.5, 
                () -> controls.getStrafe() / 1.5, 
                () -> controls.getRotation() / 1.5, 
                () -> controls.robotCentric.getAsBoolean()
            )
        );
        configureButtonBindings();
        configureTriggerBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons; the names should be self-explanitory */
        controls.zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        controls.hangExtend.whileTrue(s_Hang.runHangUp());
        controls.hangRetract.whileTrue(s_Hang.runHangDown());
        controls.hangNoLimits.onTrue(new InstantCommand(() -> s_Hang.removeHangLimits()));

//        controls.hangExtend.whileTrue(new Hang(() -> s_Hang.runHang(8));

        controls.requestAmplification.whileTrue(new InstantCommand(() -> s_Blinkin.ampLight()));
        controls.requestCoopertition.whileTrue(new InstantCommand(() -> s_Blinkin.coopertitionLight()));
        controls.runTransfer.whileTrue(new StartEndCommand(
            () -> s_Transfer.runTransfer(), 
            () -> s_Transfer.stopTransfer()
            ));
        controls.reverseTransfer.whileTrue(new StartEndCommand(
            () -> s_Transfer.reverseTransfer(), 
            () -> s_Transfer.stopTransfer()
            ));
        controls.activateShooter.whileTrue(new StartEndCommand(
            () -> s_Shooter.runShooter(), 
            () -> s_Shooter.stopShooter()));
        // controls.stopShooter.onTrue(new InstantCommand(() -> s_Shooter.stopShooter()));
        //TODO: test auto intake (collection but no transfer)
        controls.runIntake.onTrue(new IntakeCommand(s_Intake));
        controls.autoTransfer.onTrue(new MoveToTransfer(s_Transfer, s_Intake));
        
        // controls.reverseIntake.onTrue(new InstantCommand(() -> s_Intake.reverseIntake()));
        // controls.toggleIntake.onTrue(new InstantCommand(() -> s_Intake.toggleIntake()));
        controls.slowMode.whileTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> controls.getForward() / 3, 
                () -> controls.getStrafe() / 3, 
                () -> controls.getRotation() / 3, 
                () -> controls.robotCentric.getAsBoolean()
            )
        );
        controls.fastMode.whileTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> controls.getForward(), 
                () -> controls.getStrafe(), 
                () -> controls.getRotation(), 
                () -> controls.robotCentric.getAsBoolean()
            )
        );
    }

    /**
    * Use this method to define your trigger->command mappings. Triggers can be created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
    * predicate, or via the named factories in {@link}
    * 
    * Normally, configureButtonBindings and configureTriggerBindings are together in a single 
    * method called configureBindings, but separating them like this makes it easier to 
    * understand I think.
    */
    private void configureTriggerBindings() {
        s_Shooter.shooterIsSpunUp.whileTrue(new InstantCommand(() -> s_Blinkin.readyToShootLight()));
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutoRotateCommand() {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Rotate");

        return AutoBuilder.followPath(path);
    }
    public Command getAutoDriveCommand() {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Drive & Rotate");

        return AutoBuilder.followPath(path);
    }
    public Command getAutoTestCommand() {
        return new PathPlannerAuto("New Auto");
    }
    public Command autoCommandC() {
        return new PathPlannerAuto("autoC");
    }

}
