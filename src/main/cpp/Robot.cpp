/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/XboxController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <cameraserver/CameraServer.h>
#include <iostream>

class Robot : public frc::TimedRobot {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  rev::CANSparkMax::MotorType::kBrushless
   *  rev::CANSparkMax::MotorType::kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1, 2, 3 and 4. Change
   * these parameters to match your setup
   */
  static const int leftFrontDeviceID = 2, leftRearDeviceID = 3, rightFrontDeviceID = 4, rightRearDeviceID = 5;
  rev::CANSparkMax m_leftFrontMotor{leftFrontDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightFrontMotor{rightFrontDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftRearMotor{leftRearDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightRearMotor{rightRearDeviceID, rev::CANSparkMax::MotorType::kBrushed};

  /**
   * In RobotInit() below, we will group left and right motors using the MotorControllerGroup class.
   *    
   * For this reason, we only need to pass motor groups to m_robotDrive.
   * Whatever commands are sent to them will be automatically forwarded to the motors.
   */

  frc::MotorControllerGroup m_leftMotors{m_leftFrontMotor, m_leftRearMotor};
  frc::MotorControllerGroup m_rightMotors{m_rightFrontMotor, m_rightRearMotor};

  frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};

  frc::XboxController pad{0};

  bool driveMode = 0;

  // Variable speedMulFactor is the value by which the input to the motors is multiplied
  float speedMulFactor = 0.4;
  
 public:
  void RobotInit() {
    frc::CameraServer::StartAutomaticCapture();
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftFrontMotor.RestoreFactoryDefaults();
    m_rightFrontMotor.RestoreFactoryDefaults();
    m_leftRearMotor.RestoreFactoryDefaults();
    m_rightRearMotor.RestoreFactoryDefaults();

    m_rightMotors.SetInverted(true);

    std::cout << "Speed mode set to " << speedMulFactor << ", drive mode set to " << driveMode << "\n";
  }

  void TeleopPeriodic()
  {
    if (driveMode == 0)
    {
      // Drive with the TankDrive
      m_robotDrive.TankDrive(pad.GetLeftY() * speedMulFactor, pad.GetRightY() * speedMulFactor);
    }
    else
    {
        // Drive with the ArcadeDrive
        m_robotDrive.ArcadeDrive(pad.GetLeftY() * speedMulFactor, pad.GetLeftX() * speedMulFactor);
    }

    if (pad.GetAButtonPressed())
    {
        // Change the drive mode
        driveMode = !driveMode;
        std::cout << "Drive mode changed to " << driveMode << "\n";
    }

    // If one of the bumpers were pressed, change the speed
    if (pad.GetRightBumperPressed() && speedMulFactor < 1)
    {
      speedMulFactor += 0.2;
      std::cout << "Speed Mode: " << speedMulFactor << "\n";
    }
    if (pad.GetLeftBumperPressed() && speedMulFactor > 0.6)
    {
      speedMulFactor -= 0.2;
      std::cout << "Speed Mode: " << speedMulFactor << "\n";
    }
  } 
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
