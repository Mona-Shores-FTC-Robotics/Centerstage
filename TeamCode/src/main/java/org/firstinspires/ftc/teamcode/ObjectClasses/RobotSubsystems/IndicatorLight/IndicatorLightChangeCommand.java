package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.IndicatorLight;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Arm.EndEffectorSubsystem;

public class IndicatorLightChangeCommand extends CommandBase {

        // The subsystem the command runs on
        private final IndicatorLightSubsystem indicatorLightSubsystem;

        //declare target state & position
        private IndicatorLightSubsystem.LightStates targetState;

        Telemetry telemetry;

        public IndicatorLightChangeCommand(IndicatorLightSubsystem subsystem) {
            indicatorLightSubsystem = subsystem;
            //save the input state, s, as the target state


            //add the subsystem to the requirements
            addRequirements(indicatorLightSubsystem);
        }

        @Override
        public void initialize() {

            if(indicatorLightSubsystem.currentState == IndicatorLightSubsystem.LightStates.OFF) {
                indicatorLightSubsystem.red.setState(true);
                indicatorLightSubsystem.green.setState(false);
                indicatorLightSubsystem.currentState = IndicatorLightSubsystem.LightStates.RED;
            }
            if(indicatorLightSubsystem.currentState == IndicatorLightSubsystem.LightStates.RED) {
                indicatorLightSubsystem.red.setState(false);
                indicatorLightSubsystem.green.setState(true);
                indicatorLightSubsystem.currentState = IndicatorLightSubsystem.LightStates.GREEN;
            }
            if(indicatorLightSubsystem.currentState == IndicatorLightSubsystem.LightStates.GREEN) {
                indicatorLightSubsystem.red.setState(false);
                indicatorLightSubsystem.green.setState(false);
                indicatorLightSubsystem.currentState = IndicatorLightSubsystem.LightStates.AMBER;
            }
            if(indicatorLightSubsystem.currentState == IndicatorLightSubsystem.LightStates.AMBER){
                indicatorLightSubsystem.red.setState(true);
                indicatorLightSubsystem.green.setState(true);
                indicatorLightSubsystem.currentState = IndicatorLightSubsystem.LightStates.OFF;

            }

        }

        public void execute() {

        }

        @Override
        public boolean isFinished() {

                return true;
            }


    }

