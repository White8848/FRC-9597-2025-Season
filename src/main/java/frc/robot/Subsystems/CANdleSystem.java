// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


public class CANdleSystem extends SubsystemBase {
    private final CANdle m_candle1 = new CANdle(Constants.CANDLE.CANdleID1, "canivore");
    private final CANdle m_candle2 = new CANdle(Constants.CANDLE.CANdleID2, "canivore");
    private final int LedCount = 300;
    //private CommandXboxController joystick;

    private Animation m_toAnimate = null;

    public  boolean hasSetFlowEffect = false; 

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,

    }
    public Elevator m_elevator;
    private AnimationTypes m_currentAnimation;

    public boolean isflow = false;

    public CANdleSystem(Elevator elevator) {
        //this.joystick = joy;
        this.m_elevator = elevator;
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle1.configAllSettings(configAll, 100);
        m_candle2.configAllSettings(configAll, 100);
    }

    public void incrementAnimation() {
        m_elevator.m_current_state = Constants.Elevator.State.FLOW;
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        m_elevator.m_current_state = Constants.Elevator.State.FLOW;
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void Changecolor(Constants.Elevator.State state) {
        m_elevator.m_current_state = state;//将颜色变为装逼灯
 
    }



    public void setOff() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(0, 0, 0);
        m_candle2.setLEDs(0, 0, 0);

    }
    

    public void setWhite() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 255, 255);
        m_candle2.setLEDs(255, 255, 255);
        
    }
    public void setRed() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 0, 0);
        m_candle2.setLEDs(255, 0, 0);

    }
    public void setGreen() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(0, 255, 0);
        m_candle2.setLEDs(0, 255, 0);
 
    }
    public void setBlue() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(0, 0, 255);
        m_candle2.setLEDs(0, 0, 255);
    }
    public void setYellow(){ 
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 255, 0);
        m_candle2.setLEDs(255, 255, 0);

    }
    public void setCyan() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(0, 255, 255);
        m_candle2.setLEDs(0, 255, 255);
    }
    public void setMagenta() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 0, 255);
        m_candle2.setLEDs(255, 0, 255);

    }
    public void setOrange() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 165, 0);
        m_candle2.setLEDs(255, 165, 0);

    }



    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat1() { return m_candle1.getBusVoltage(); }
    public double getVbat2() { return m_candle2.getBusVoltage(); }

    public double get5V1() { return m_candle1.get5VRailVoltage(); }
    public double get5V2() { return m_candle2.get5VRailVoltage(); }

    public double getCurrent1() { return m_candle1.getCurrent(); }
    public double getCurrent2() { return m_candle2.getCurrent(); }

    public double getTemperature1() { return m_candle1.getTemperature(); }
    public double getTemperature2() { return m_candle2.getTemperature(); }

    public void configBrightness(double percent) { m_candle1.configBrightnessScalar(percent, 0);
                                                   m_candle2.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle1.configLOSBehavior(disableWhenLos, 0); 
                                                    m_candle2.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle1.configLEDType(type, 0); 
                                                   m_candle2.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle1.configStatusLedState(offWhenActive, 0); 
                                                                 m_candle2.configStatusLedState(offWhenActive, 0); }



    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
  
        }

    }

    @Override
    public void periodic() {
        //System.out.println("m_currentstate: " + m_elevator.m_current_state);
        switch(m_elevator.m_current_state) {
            case START:
                setOff();
                break;
            case INTAKE:
                setRed();
                break;
            case REEF_2:
                setGreen();
                break;
            case REEF_3:
                setBlue();
                break;
            case REEF_4:
                setYellow();
                break;
            case GETBALL1:
                setCyan();
                break;
            case GETBALL2:
                setMagenta();
                break;
            case BARGE:
                setWhite();
                break;
            case FLOW:
                m_candle1.animate(new RainbowAnimation(1, 1, LedCount));//single showoff lights
                m_candle2.animate(new RainbowAnimation(1, 1, LedCount));
                break;
            default:
                setOff();
                break;
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
