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
export const allSensors = Object.values(Sensor);

export interface SensorConfig {
  displayName: string;
  topic: string;
}
