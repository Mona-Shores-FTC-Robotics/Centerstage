package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class ShoulderSubsystem extends SubsystemBase {

    public static class ShoulderParameters20245 {

        public double INTAKE_REST = .5;
        public double INTAKE_VALUE = 0.59;
        public double STARTING_POSITION = 0.59;
        public double BACKDROP_VALUE = .195;
        public double HALFWAY = .35;
        public double INTAKE_VALUE_STAGING=.45;
    }

    public static ShoulderParameters20245 shoulderParameters20245 = new ShoulderParameters20245();

    public static class ShoulderParameters19429 {

        public double INTAKE_REST = .5;
        public double INTAKE_VALUE = 0.53;
        public double INTAKE_VALUE_STAGING = 0.45;
        public double STARTING_POSITION = .6;
        public double BACKDROP_VALUE = .195;
        public double HALFWAY = .4;
    }

    public static ShoulderParameters19429 shoulderParameters19429 = new ShoulderParameters19429();

    public enum ShoulderStates {
        REST (.5),
        INTAKE (0.53),
        HALFWAY(.4),
        BACKDROP (.195),
        INTAKE_VALUE_STAGING (0.45),
        STARTING_POSITION (.53);

        public double position;
        ShoulderStates(double p) {
            this.position = p;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public Servo shoulder;
    public ShoulderStates currentState;

    public ShoulderSubsystem(final HardwareMap hMap, final String name) {
        shoulder = hMap.servo.get("shoulder");
    }

    //todo make sure this gets merged
    public void init() {
        SetRobotSpecificParameters();
        currentState= ShoulderStates.STARTING_POSITION;
        shoulder.setPosition(currentState.position);
    }

    public void initTele() {
        SetRobotSpecificParameters();
        currentState= ShoulderStates.INTAKE;
        shoulder.setPosition(currentState.position);
    }


    private void SetRobotSpecificParameters() {
        if (MatchConfig.robot19429){
            ShoulderStates.INTAKE.SetState(shoulderParameters19429.INTAKE_VALUE);
            ShoulderStates.BACKDROP.SetState(shoulderParameters19429.BACKDROP_VALUE);
            ShoulderStates.HALFWAY.SetState(shoulderParameters19429.HALFWAY);
            ShoulderStates.STARTING_POSITION.SetState(shoulderParameters19429.STARTING_POSITION);

        } else{
            ShoulderStates.INTAKE.SetState(shoulderParameters20245.INTAKE_VALUE);
            ShoulderStates.BACKDROP.SetState(shoulderParameters20245.BACKDROP_VALUE);
            ShoulderStates.HALFWAY.SetState(shoulderParameters20245.HALFWAY);
            ShoulderStates.STARTING_POSITION.SetState(shoulderParameters20245.STARTING_POSITION);
        }
    }


    public void periodic(){
        SetRobotSpecificParameters();
        //Add the Shoulder State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Shoulder State", currentState);
    }

    public void setCurrentState(ShoulderStates state) {
        currentState=state;
    }
}
