// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.game.rebuilt2026.*;
import frc.robot.util.Elastic;
import java.io.FileReader;
import java.util.Properties;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;


public class RobotContainer {
  public RobotConfig robotConfig;

  public AnalogInput ultrasonicSensor = new AnalogInput(0);

  public RobotContainer() {
    // Load robot name from configuration file
    // Check if the robot is running in simulation
    Properties robotProperties = new Properties();
   

    // get configuration from robot_config.properties
    try {
      String configPath = "";
      if (Robot.isReal()) {
        configPath = "/home/lvuser/deploy";
      } else {
        configPath = "src/main/deploy";
      }
      FileReader propertiesFile = new FileReader(configPath + "/robot_config.properties");
      robotProperties.load(propertiesFile);
      // System.out.println(simulationProperties.toString());
      propertiesFile.close();
    } catch (Exception e) {
      System.out.println(e);
    }

    String robotName = robotProperties.getProperty("robot.name", "UNKNOWN");
    RobotConfig robotConfig = new RobotConfig(robotProperties);
    // System.out.println(robotName + ", " + robotDrive);

    // try (FileInputStream input = new FileInputStream("simulation.properties")) {
    //   robotProperties.load(input);b
    // } catch (IOException e) {
    //   System.err.println("Failed to load simulation configuration file: " + e.getMessage());
    //   System.exit(1);
    // }

    Preferences.initString("Robot Name", robotName);
    robotName = Preferences.getString("Robot Name", robotName);

    System.out.println("Loading Settings for Robot Name = " + robotName);
    Elastic.sendNotification(
        new Elastic.Notification()
            .withDescription("Loading Settings for Robot Name = " + robotName));

    robotConfig.configureBindings();
  }

  public Command getAutonomousCommand() {
    return robotConfig.autoChooser.getSelected();
  }
}
