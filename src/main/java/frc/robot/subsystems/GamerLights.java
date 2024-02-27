package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Spark;

//Manual: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf 

  public class GamerLights extends SubsystemBase {

    Spark blinkin = new Spark(0);

    /* FOR FUTURE PURPOSES
    *  If needing to press a button on the controller, to change a LED LIGHT
    */

    Joystick driverController = new Joystick(0);

    public Blinkin() {}
    
    public void NAME() { blinkin.set(COLOR); }

    //IF NEED COLOR DURING DRIVING
    public void driveLight() {
        blinkin.set(0.69);
    } 

    //IF NEED COLOR DURING BUTTON PRESS 
    public void buttonLight() {
      blinkin.set(0.69);
    } 

    public void idleLight() { blinkin.set(0.85); }

    /* FOR FUTURE PURPOSES (IF YOU NEED A COLOR DURING DRIVING)
    Set name if using, set button id (X)*/
  @Override
  public void periodic() {
    if(driverController.getRawButton(2)){
      driveLight();
    }
    else if(driverController.getRawButton(3)){
      buttonLight();
    }
    else {
      idleLight();
    }
  }
}
    