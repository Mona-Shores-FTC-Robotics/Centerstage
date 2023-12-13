package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public final class GripperSubsystem extends SubsystemBase {

    public static class GripperParameters20245{

        public GripperStates GRIPPER_STARTING_STATE = GripperStates.OPEN;
        public double REST_POSITION = .5;
        public double OPEN_POSITION = 0.45;
        public double ONE_PIXEL_RELEASE_POSITION=0.49;
        public double CLOSED_POSITION = .55;
    }
    public static GripperSubsystem.GripperParameters20245 gripperParameters20245 = new GripperSubsystem.GripperParameters20245();

    public static class GripperParameters19429{

        public GripperStates GRIPPER_STARTING_STATE = GripperStates.OPEN;
        public double REST_POSITION = .5;
        public double OPEN_POSITION = .45;
        public double ONE_PIXEL_RELEASE_POSITION=0.49;
        public double CLOSED_POSITION = .55;
    }
    public static GripperSubsystem.GripperParameters19429 gripperParameters19429 = new GripperSubsystem.GripperParameters19429();


    public enum GripperStates {
        REST (0.5),
        CLOSED (.55),
        OPEN (0.45),
        ONE_PIXEL_RELEASE_POSITION (0.49);
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
        SetRobotSpecificParameters();
        currentState= gripperParameters20245.GRIPPER_STARTING_STATE;
        endEffector.setPosition(currentState.position);
    }
    private void SetRobotSpecificParameters() {
        if (MatchConfig.robot19429){
            GripperStates.ONE_PIXEL_RELEASE_POSITION.SetState(GripperSubsystem.gripperParameters19429.ONE_PIXEL_RELEASE_POSITION);
            GripperStates.OPEN.SetState(GripperSubsystem.gripperParameters19429.OPEN_POSITION);
            GripperStates.CLOSED.SetState(GripperSubsystem.gripperParameters19429.CLOSED_POSITION);

        } else{
            GripperStates.ONE_PIXEL_RELEASE_POSITION.SetState(GripperSubsystem.gripperParameters20245.ONE_PIXEL_RELEASE_POSITION);
            GripperStates.OPEN.SetState(GripperSubsystem.gripperParameters20245.OPEN_POSITION);
            GripperStates.CLOSED.SetState(GripperSubsystem.gripperParameters20245.CLOSED_POSITION);
        }
    }
    public void periodic(){
        SetRobotSpecificParameters();
        //Add the Gripper State to our loop telemetry packet
        MatchConfig.telemetryPacket.put("Gripper State", currentState);
    }

}
