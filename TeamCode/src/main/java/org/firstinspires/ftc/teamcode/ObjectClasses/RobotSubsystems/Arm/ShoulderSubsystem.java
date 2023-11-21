package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake.IntakeSubsystem;

@Config
public class ShoulderSubsystem extends SubsystemBase {

    public static class ShoulderParameters {

        public double INTAKE_REST = .5;
        public double INTAKE_VALUE = .55;
        public double STARTING_POSITION = .6;
        public double BACKDROP_VALUE = .2;
        public double HALFWAY = .4;
    }

    public static ShoulderParameters shoulderParameters = new ShoulderParameters();

    public enum ShoulderStates {
        REST (.5),
        INTAKE (.55),
        HALFWAY(.4),
        BACKDROP (.2),
        STARTING_POSITION (.6);

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

    public void init() {
        currentState= ShoulderStates.STARTING_POSITION;
        shoulder.setPosition(currentState.position);
    }

    public void initTele() {
        currentState= ShoulderStates.INTAKE;
        shoulder.setPosition(currentState.position);
    }
    public void periodic(){
        ShoulderStates.INTAKE.SetState(shoulderParameters.INTAKE_VALUE);
        ShoulderStates.BACKDROP.SetState(shoulderParameters.BACKDROP_VALUE);
        ShoulderStates.HALFWAY.SetState(shoulderParameters.HALFWAY);
        ShoulderStates.STARTING_POSITION.SetState(shoulderParameters.STARTING_POSITION);

        //Add the Shoulder State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Shoulder State", currentState);
    }

    public void setCurrentState(ShoulderStates state) {
        currentState=state;
    }
}
