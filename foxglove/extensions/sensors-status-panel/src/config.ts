import { Robot } from "@duke-robotics/robot-name";
import { secToNsec } from "@duke-robotics/utils";

export enum Sensor {
  IMU = "IMU",
  DVL = "DVL",
  Pressure = "Pressure",
  Gyro = "Gyro",
  FrontDAI = "FrontDAI",
  FrontMono = "FrontMono",
  BottomMono = "BottomMono",
  Sonar = "Sonar",
  IVCModem = "IVCModem",
}
export const SENSORS = Object.values(Sensor);

export interface SensorConfig {
  displayName: string;
  topic: string;
}

/**
 * Configuration data for all sensors (across all robots).
 */
export const SENSOR_CONFIG: Record<Sensor, SensorConfig> = {
  [Sensor.DVL]: { displayName: "DVL", topic: "/sensors/dvl/raw" },
  [Sensor.IMU]: { displayName: "IMU", topic: "/vectornav/imu" },
  [Sensor.Pressure]: { displayName: "Pressure", topic: "/sensors/depth" },
  [Sensor.Gyro]: { displayName: "Gyro", topic: "/sensors/gyro/status" },
  [Sensor.FrontDAI]: { displayName: "Front DAI", topic: "/camera/front/rgb/preview/compressed" },
  [Sensor.FrontMono]: { displayName: "Front Mono", topic: "/camera/usb/front/compressed" },
  [Sensor.BottomMono]: { displayName: "Bottom Mono", topic: "/camera/usb/bottom/compressed" },
  [Sensor.Sonar]: { displayName: "Sonar", topic: "/sonar/status" },
  [Sensor.IVCModem]: { displayName: "IVC Modem", topic: "/sensors/modem/status" },
};

/**
 * Mapping of each robot to the sensors we want to monitor.
 */
export const ROBOT_CONFIG: Record<Robot, Array<Sensor>> = {
  [Robot.Oogway]: [Sensor.DVL, Sensor.IMU, Sensor.Pressure, Sensor.Gyro, Sensor.FrontDAI, Sensor.Sonar],
  [Robot.Crush]: [Sensor.FrontMono, Sensor.BottomMono, Sensor.Sonar, Sensor.IVCModem, Sensor.IMU],
};

export const TOPIC_TO_SENSOR: Record<string, Sensor> = Object.fromEntries(
  SENSORS.map((sensor) => {
    const config = SENSOR_CONFIG[sensor];
    return [config.topic, sensor];
  }),
);

/**
 * Time until sensor is considered down.
 */
export const SENSOR_DOWN_THRESHOLD_NSEC = secToNsec(1);
