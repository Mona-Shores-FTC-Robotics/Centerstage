package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShoulderSubsystem extends SubsystemBase {



    public static class ShoulderParameters {
        public double SHOULDER_ROTATE_THRESHOLD_MILLISECONDS = 100;
        public double INTAKE_VALUE = .55;
        public double STARTING_POSITION = .7;
        public double BACKDROP_VALUE = .2;
        public double HALFWAY = .4;
    }

    public static ShoulderParameters shoulderParameters = new ShoulderParameters();

    public enum ShoulderStates {
        INTAKE (.55),
        HALFWAY(.4),
        BACKDROP (.2),
        STARTING_POSITION (.7);

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
    public ShoulderStates targetState;
    public double currentPosition;
    private TelemetryPacket telemetryPacket;

    public ShoulderSubsystem(final HardwareMap hMap, final String name) {
        shoulder = hMap.servo.get("shoulder");
    }

    public void init() {
        ShoulderStates.BACKDROP.SetState(shoulderParameters.BACKDROP_VALUE);
        ShoulderStates.INTAKE.SetState(shoulderParameters.INTAKE_VALUE);
        currentState= ShoulderStates.STARTING_POSITION;
        currentPosition = currentState.position;
        shoulder.setPosition(currentPosition);
    }

    public void periodic(){
        ShoulderStates.INTAKE.SetState(shoulderParameters.INTAKE_VALUE);
        ShoulderStates.BACKDROP.SetState(shoulderParameters.BACKDROP_VALUE);
        ShoulderStates.HALFWAY.SetState(shoulderParameters.HALFWAY);
        ShoulderStates.STARTING_POSITION.SetState(shoulderParameters.STARTING_POSITION);

        telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Current Shoulder State", currentState);

        if (targetState!=currentState) {
            telemetryPacket.put("Target Shoulder State", targetState);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
    }


    public void setTargetState(ShoulderStates state) {
        targetState=state;
    }

    public void setCurrentState(ShoulderStates state) {
        currentState=state;
    }
}
