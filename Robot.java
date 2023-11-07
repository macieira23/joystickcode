// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  Joystick joy = new Joystick(0);
  VictorSPX m_Left1 = new VictorSPX(2);
  VictorSPX m_Left2 = new VictorSPX(4);
  VictorSPX m_Right1 = new VictorSPX(1);
  VictorSPX m_Right2 = new VictorSPX(3);

  boolean bntA, bntB, bntX, bntY;
  double gatilhorigth, gatilholeft;
  double spdbutton = 0;
  double spdaxis = 0;
  double spdfinal;
  double speed_L = 0;
  double speed_R = 0;
  double Y1, X1;
  double X2, Y2;
  double seno, seno2;
  double mag, mag2;
  double value_right = 0;
  double value_left = 0;
  int pov;
  double mL = 0;
  double mR = 0;
  boolean analog1 = true;
  boolean analog2 = true;

  @Override
  public void robotInit() {
    m_Right2.follow(m_Right1);
    m_Left2.follow(m_Left1);
    m_Left1.setInverted(true);
  }

  
@Override
  public void teleopPeriodic() {
    pov = joy.getPOV(0);
    bntA = joy.getRawButton(1);
    bntB = joy.getRawButton(2);
    bntX = joy.getRawButton(3);
    bntY = joy.getRawButton(4);
    gatilhorigth = joy.getRawAxis(3);
    gatilholeft = joy.getRawAxis(2);
    Y1 = -joy.getRawAxis(1);
    X1 = joy.getRawAxis(0);
    Y2 = -joy.getRawAxis(5);
    X2 = joy.getRawAxis(4);

    mag = Math.hypot(X1, Y1);
    seno = Y1 / mag;
   
    mag2 = Math.hypot(X2, Y2);
    seno2 = Y2 / mag;

    if ((Math.abs(X1) > 0.04) || (Math.abs(Y1) > 0.04)) {
      analog1 = true;
    } else {
      analog1 = false;
    }
    if ((Math.abs(X2) > 0.04) || (Math.abs(Y2) > 0.04)) {
      analog2 = true;
    } else {
      analog2 = false;
    }
    if (analog1 == false && analog2 == false) {
      value_left = 0.0;
      value_right = 0.0;
    }

    if (joy.getRawButton(1)) {
      spdbutton = 0.50;
    } else if (joy.getRawButton(2)) {
      spdbutton = 0.25;
    } else if (joy.getRawButton(3)) {
      spdbutton = 1;
    }

    if (gatilhorigth > 0) {
      spdaxis = gatilhorigth;
    } else if (gatilholeft >= 0) {
      spdaxis = -gatilholeft;
    }

    spdfinal = spdbutton * spdaxis;

    if (gatilhorigth > 0) {
      m_Left1.set(ControlMode.PercentOutput, spdfinal);
      m_Right1.set(ControlMode.PercentOutput, -spdfinal);
    }

    if (gatilholeft > 0) {
      m_Left1.set(ControlMode.PercentOutput, -spdfinal);
      m_Right1.set(ControlMode.PercentOutput, spdfinal);
    }

    if (analog1) {
      // QUADRANTE 1 ALTERAÇÃO (PARA FRENTE)
      if (X1 > 0 && Y1 > 0) {
        value_left = mag;
        value_right = (2 * seno - 1)*mag;

      }
      // QUADRANTE 2 (DIREITA)
      else if (X1 < 0 && Y1 > 0) {
        value_left = (2 * seno - 1)*mag;
        value_right = mag;

      }
      // QUADRANTE 3 ALTERAÇÃO (PARA TRAS)
      else if (X1 < 0 && Y1 < 0) {
        value_left = -mag;
        value_right = (2 * seno + 1)*mag;
      }
      // QUADRANTE 4(ESQUERDA)
      else if (X1 > 0 && Y1 < 0) {
        value_left = (2 * seno + 1)*mag;
        value_right = -mag;
      }

    }

    if (analog2) {
      // QUADRANTE 1 FRENTE
      if (X2 > 0 && Y2 > 0) {
        value_left = (-2 * seno2 + 1) *mag2;
        value_right = -mag2;
      }

      // QUADRANTE 2 TRAS
      if (X2 < 0 && Y2 < 0) {
        value_left= mag2;
        value_right = (-2 * seno2-1)*mag2;
      }

      // QUADRANTE 3 esquerda
      else if (X2 > 0 && Y2 < 0) {
        value_left = -mag2;
        value_right =  (-2*seno2+1) *mag2;
      }

      // QUADRANTE 4 direita
      else if (X2 < 0 && Y2 > 0) {
        value_left= mag2;
        value_right = (-2*seno2-1)*mag2;
      }   
     
    }


    speed_L = spdbutton*value_left;
    speed_R = spdbutton*value_right;
   
    m_Left1.set(ControlMode.PercentOutput, speed_L);
    m_Right1.set(ControlMode.PercentOutput, speed_R);
    
    if (spdbutton < Math.abs(speed_L)) {
      speed_L = Math.copySign(spdbutton, speed_L);
    }
    if (spdbutton < Math.abs(speed_R)) {
      speed_R = Math.copySign(spdbutton, speed_R);
    }

    
    switch (pov) {
      case 0:
        mL = 0.40;
        mR = 0.40;
        break;

      case 45:
        mL = 0.40;
        mR = 0;
        break;

      case 90:
        mL = 0.40;
        mR = -0.40;
        break;

      case 135:
        mL = 0;
        mR = -0.40;
        break;

      case 180:
        mL = -0.40;
        mR = -0.40;
        break;

      case 225:
        mL = 0;
        mR = 0.40;
        break;

      case 270:
        mL = -0.40;
        mR = 0.40;
        break;

      case 315:
        mL = 0;
        mR = 0.40;
        break;

      default:
      case -1:
        mL = 0;
        mR = 0;
    }

    SmartDashboard.putBoolean("botao (A)  ativado", bntA);
    SmartDashboard.putBoolean(" botao  (B)  ativado", bntB);
    SmartDashboard.putBoolean(" botao  (X) ativado", bntX);
    SmartDashboard.putBoolean(" botao (Y)  ativado", bntY);
    SmartDashboard.putNumber("gatilho direita", gatilhorigth * spdbutton);
    SmartDashboard.putNumber("gatilho esquerda", gatilholeft * spdbutton);
    SmartDashboard.putNumber("m_left", speed_L);
    SmartDashboard.putNumber("m_right", speed_R);
    SmartDashboard.putNumber("pov", pov);
    SmartDashboard.putNumber("spdbtn", spdbutton);
    SmartDashboard.putBoolean("analog1", analog1);
    SmartDashboard.putBoolean("analog2", analog2);

  }

}

