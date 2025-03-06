package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.CANdleSystem;

public class CANdlePrintCommands {
    static public class PrintVBat1 extends InstantCommand {
        public PrintVBat1(CANdleSystem candleSystem) {
            super(() -> System.out.println("Vbat is " + candleSystem.getVbat1() + "V"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class PrintVBat2 extends InstantCommand {
        public PrintVBat2(CANdleSystem candleSystem) {
            super(() -> System.out.println("Vbat is " + candleSystem.getVbat2() + "V"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class Print5V1 extends InstantCommand {
        public Print5V1(CANdleSystem candleSystem) {
            super(() -> System.out.println("5V is " + candleSystem.get5V1() + "V"), candleSystem);
        }
        
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }

    static public class Print5V2 extends InstantCommand {
        public Print5V2(CANdleSystem candleSystem) {
            super(() -> System.out.println("5V is " + candleSystem.get5V2() + "V"), candleSystem);
        }
        
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }

    static public class PrintCurrent1 extends InstantCommand {
        public PrintCurrent1(CANdleSystem candleSystem) {
            super(() -> System.out.println("Current is " + candleSystem.getCurrent1() + "A"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }

    static public class PrintCurrent2 extends InstantCommand {
        public PrintCurrent2(CANdleSystem candleSystem) {
            super(() -> System.out.println("Current is " + candleSystem.getCurrent2() + "A"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class PrintTemperature1 extends InstantCommand {
        public PrintTemperature1(CANdleSystem candleSystem) {
            super(() -> System.out.println("Temperature is " + candleSystem.getTemperature1() + "C"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    static public class PrintTemperature2 extends InstantCommand {
        public PrintTemperature2(CANdleSystem candleSystem) {
            super(() -> System.out.println("Temperature is " + candleSystem.getTemperature1() + "C"), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
}
