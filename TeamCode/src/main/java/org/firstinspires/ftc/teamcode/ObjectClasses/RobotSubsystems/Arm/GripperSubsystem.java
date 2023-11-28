package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public final class GripperSubsystem extends SubsystemBase {

    public static GripperStates GRIPPER_STARTING_STATE = GripperStates.OPEN;
    public static double REST_POSITION = .5;
    public static double OPEN_POSITION = .45;
    public static double ONE_PIXEL_RELEASE_POSITION=.495; //infraed is .507  //ultraviolet needs at least .497 -doing .495 to be safe.
    public static double CLOSED_POSITION = .55;

    public enum GripperStates {
        REST (.5),
        CLOSED (.55),
        OPEN (.45),
        ONE_PIXEL_RELEASE_POSITION (.52);
        public double position;
        GripperStates(double pos) {
            this.position = pos;
        }
        void SetState(double pos){
            this.position = pos;
        }
    }

    public final Servo endEffector;
    public GripperStates currentState;

    public GripperSubsystem(final HardwareMap hMap, final String name) {
        endEffector = hMap.get(Servo.class, name);
    }

    public void init() {
        currentState= GRIPPER_STARTING_STATE;
        endEffector.setPosition(currentState.position);
    }

    public void periodic(){
        GripperStates.CLOSED.SetState(CLOSED_POSITION);
        GripperStates.OPEN.SetState(OPEN_POSITION);
        GripperStates.ONE_PIXEL_RELEASE_POSITION.SetState(ONE_PIXEL_RELEASE_POSITION);
        //Add the Gripper State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Gripper State", currentState);
    }

}
