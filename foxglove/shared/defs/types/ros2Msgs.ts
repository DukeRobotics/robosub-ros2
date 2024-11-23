/* eslint-disable */
// These files were generated using "ros-typescript-generator"
export enum StatisticsMsgsStatisticDataTypeConst {
  STATISTICS_DATA_TYPE_UNINITIALIZED = 0,
  STATISTICS_DATA_TYPE_AVERAGE = 1,
  STATISTICS_DATA_TYPE_MINIMUM = 2,
  STATISTICS_DATA_TYPE_MAXIMUM = 3,
  STATISTICS_DATA_TYPE_STDDEV = 4,
  STATISTICS_DATA_TYPE_SAMPLE_COUNT = 5,
}

export interface ActionMsgsCancelGoalResponse {
  return_code: number;
}

export enum ActionMsgsCancelGoalResponseConst {
  ERROR_NONE = 0,
  ERROR_REJECTED = 1,
  ERROR_UNKNOWN_GOAL_ID = 2,
  ERROR_GOAL_TERMINATED = 3,
}

export interface ActionMsgsGoalInfo {
  goal_id: UniqueIdentifierMsgsUuid;
  stamp: { sec: number, nanosec: number };
}

export interface ActionMsgsGoalStatus {
  status: number;
}

export enum ActionMsgsGoalStatusConst {
  STATUS_UNKNOWN = 0,
  STATUS_ACCEPTED = 1,
  STATUS_EXECUTING = 2,
  STATUS_CANCELING = 3,
  STATUS_SUCCEEDED = 4,
  STATUS_CANCELED = 5,
  STATUS_ABORTED = 6,
}

export interface ActionlibMsgsGoalId {
  stamp: { sec: number, nanosec: number };
  id: string;
}

export interface ActionlibMsgsGoalStatus {
  status: number;
  text: string;
}

export enum ActionlibMsgsGoalStatusConst {
  PENDING = 0,
  ACTIVE = 1,
  PREEMPTED = 2,
  SUCCEEDED = 3,
  ABORTED = 4,
  REJECTED = 5,
  PREEMPTING = 6,
  RECALLING = 7,
  RECALLED = 8,
  LOST = 9,
}

export interface ActionlibMsgsGoalStatusArray {
  header: StdMsgsHeader;
}

export interface DiagnosticMsgsAddDiagnosticsRequest {
  load_namespace: string;
}

export interface DiagnosticMsgsAddDiagnosticsResponse {
  success: boolean;
  message: string;
}

export interface DiagnosticMsgsDiagnosticArray {
  header: StdMsgsHeader;
}

export interface DiagnosticMsgsDiagnosticStatus {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
}

export enum DiagnosticMsgsDiagnosticStatusConst {
  OK = 0,
  WARN = 1,
  ERROR = 2,
  STALE = 3,
}

export interface DiagnosticMsgsKeyValue {
  key: string;
  value: string;
}

export interface DiagnosticMsgsSelfTestResponse {
  id: string;
  passed: number;
}

export interface GeometryMsgsAccelStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsAccelWithCovariance {
  covariance: number[];
}

export interface GeometryMsgsAccelWithCovarianceStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsInertia {
  m: number;
  com: GeometryMsgsVector3;
  ixx: number;
  ixy: number;
  ixz: number;
  iyy: number;
  iyz: number;
  izz: number;
}

export interface GeometryMsgsInertiaStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsPoint {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsPoint32 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsPointStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsPolygonInstance {
  polygon: GeometryMsgsPolygon;
  id: number;
}

export interface GeometryMsgsPolygonInstanceStamped {
  header: StdMsgsHeader;
  polygon: GeometryMsgsPolygonInstance;
}

export interface GeometryMsgsPolygonStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsPose2D {
  x: number;
  y: number;
  theta: number;
}

export interface GeometryMsgsPoseArray {
  header: StdMsgsHeader;
}

export interface GeometryMsgsPoseStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsPoseWithCovariance {
  covariance: number[];
}

export interface GeometryMsgsPoseWithCovarianceStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsQuaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface GeometryMsgsQuaternionStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsTransformStamped {
  header: StdMsgsHeader;
  child_frame_id: string;
}

export interface GeometryMsgsTwistStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsTwistWithCovariance {
  covariance: number[];
}

export interface GeometryMsgsTwistWithCovarianceStamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsVector3 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsVector3Stamped {
  header: StdMsgsHeader;
}

export interface GeometryMsgsVelocityStamped {
  header: StdMsgsHeader;
  body_frame_id: string;
  reference_frame_id: string;
}

export interface GeometryMsgsWrenchStamped {
  header: StdMsgsHeader;
}

export interface LifecycleMsgsChangeStateResponse {
  success: boolean;
}

export interface LifecycleMsgsState {
  id: number;
  label: string;
}

export enum LifecycleMsgsStateConst {
  PRIMARY_STATE_UNKNOWN = 0,
  PRIMARY_STATE_UNCONFIGURED = 1,
  PRIMARY_STATE_INACTIVE = 2,
  PRIMARY_STATE_ACTIVE = 3,
  PRIMARY_STATE_FINALIZED = 4,
  TRANSITION_STATE_CONFIGURING = 10,
  TRANSITION_STATE_CLEANINGUP = 11,
  TRANSITION_STATE_SHUTTINGDOWN = 12,
  TRANSITION_STATE_ACTIVATING = 13,
  TRANSITION_STATE_DEACTIVATING = 14,
  TRANSITION_STATE_ERRORPROCESSING = 15,
}

export interface LifecycleMsgsTransition {
  id: number;
  label: string;
}

export enum LifecycleMsgsTransitionConst {
  TRANSITION_CREATE = 0,
  TRANSITION_CONFIGURE = 1,
  TRANSITION_CLEANUP = 2,
  TRANSITION_ACTIVATE = 3,
  TRANSITION_DEACTIVATE = 4,
  TRANSITION_UNCONFIGURED_SHUTDOWN = 5,
  TRANSITION_INACTIVE_SHUTDOWN = 6,
  TRANSITION_ACTIVE_SHUTDOWN = 7,
  TRANSITION_DESTROY = 8,
  TRANSITION_ON_CONFIGURE_SUCCESS = 10,
  TRANSITION_ON_CONFIGURE_FAILURE = 11,
  TRANSITION_ON_CONFIGURE_ERROR = 12,
  TRANSITION_ON_CLEANUP_SUCCESS = 20,
  TRANSITION_ON_CLEANUP_FAILURE = 21,
  TRANSITION_ON_CLEANUP_ERROR = 22,
  TRANSITION_ON_ACTIVATE_SUCCESS = 30,
  TRANSITION_ON_ACTIVATE_FAILURE = 31,
  TRANSITION_ON_ACTIVATE_ERROR = 32,
  TRANSITION_ON_DEACTIVATE_SUCCESS = 40,
  TRANSITION_ON_DEACTIVATE_FAILURE = 41,
  TRANSITION_ON_DEACTIVATE_ERROR = 42,
  TRANSITION_ON_SHUTDOWN_SUCCESS = 50,
  TRANSITION_ON_SHUTDOWN_FAILURE = 51,
  TRANSITION_ON_SHUTDOWN_ERROR = 52,
  TRANSITION_ON_ERROR_SUCCESS = 60,
  TRANSITION_ON_ERROR_FAILURE = 61,
  TRANSITION_ON_ERROR_ERROR = 62,
  TRANSITION_CALLBACK_SUCCESS = 97,
  TRANSITION_CALLBACK_FAILURE = 98,
  TRANSITION_CALLBACK_ERROR = 99,
}

export interface LifecycleMsgsTransitionEvent {
  timestamp: number;
}

export interface NavMsgsGetPlanRequest {
  start: GeometryMsgsPoseStamped;
  goal: GeometryMsgsPoseStamped;
  tolerance: number;
}

export interface NavMsgsGridCells {
  header: StdMsgsHeader;
  cell_width: number;
  cell_height: number;
  cells: GeometryMsgsPoint[];
}

export interface NavMsgsLoadMapRequest {
  map_url: string;
}

export interface NavMsgsLoadMapResponse {
  map: NavMsgsOccupancyGrid;
  result: number;
}

export enum NavMsgsLoadMapResponseConst {
  RESULT_SUCCESS = 0,
  RESULT_MAP_DOES_NOT_EXIST = 1,
  RESULT_INVALID_MAP_DATA = 2,
  RESULT_INVALID_MAP_METADATA = 3,
  RESULT_UNDEFINED_FAILURE = 255,
}

export interface NavMsgsMapMetaData {
  map_load_time: { sec: number, nanosec: number };
  resolution: number;
  width: number;
  height: number;
  origin: GeometryMsgsPose;
}

export interface NavMsgsOccupancyGrid {
  header: StdMsgsHeader;
  data: number[];
}

export interface NavMsgsOdometry {
  header: StdMsgsHeader;
  child_frame_id: string;
  pose: GeometryMsgsPoseWithCovariance;
  twist: GeometryMsgsTwistWithCovariance;
}

export interface NavMsgsPath {
  header: StdMsgsHeader;
  poses: GeometryMsgsPoseStamped[];
}

export interface NavMsgsSetMapRequest {
  map: NavMsgsOccupancyGrid;
  initial_pose: GeometryMsgsPoseWithCovarianceStamped;
}

export interface NavMsgsSetMapResponse {
  success: boolean;
}

export interface RosgraphMsgsClock {
  clock: { sec: number, nanosec: number };
}

export interface SensorMsgsBatteryState {
  header: StdMsgsHeader;
  voltage: number;
  temperature: number;
  current: number;
  charge: number;
  capacity: number;
  design_capacity: number;
  percentage: number;
  power_supply_status: number;
  power_supply_health: number;
  power_supply_technology: number;
  present: boolean;
  cell_voltage: number[];
  cell_temperature: number[];
  location: string;
  serial_number: string;
}

export enum SensorMsgsBatteryStateConst {
  POWER_SUPPLY_STATUS_UNKNOWN = 0,
  POWER_SUPPLY_STATUS_CHARGING = 1,
  POWER_SUPPLY_STATUS_DISCHARGING = 2,
  POWER_SUPPLY_STATUS_NOT_CHARGING = 3,
  POWER_SUPPLY_STATUS_FULL = 4,
  POWER_SUPPLY_HEALTH_UNKNOWN = 0,
  POWER_SUPPLY_HEALTH_GOOD = 1,
  POWER_SUPPLY_HEALTH_OVERHEAT = 2,
  POWER_SUPPLY_HEALTH_DEAD = 3,
  POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4,
  POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5,
  POWER_SUPPLY_HEALTH_COLD = 6,
  POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7,
  POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8,
  POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0,
  POWER_SUPPLY_TECHNOLOGY_NIMH = 1,
  POWER_SUPPLY_TECHNOLOGY_LION = 2,
  POWER_SUPPLY_TECHNOLOGY_LIPO = 3,
  POWER_SUPPLY_TECHNOLOGY_LIFE = 4,
  POWER_SUPPLY_TECHNOLOGY_NICD = 5,
  POWER_SUPPLY_TECHNOLOGY_LIMN = 6,
  POWER_SUPPLY_TECHNOLOGY_TERNARY = 7,
  POWER_SUPPLY_TECHNOLOGY_VRLA = 8,
}

export interface SensorMsgsCameraInfo {
  header: StdMsgsHeader;
  height: number;
  width: number;
  distortion_model: string;
  d: number[];
  k: number[];
  r: number[];
  p: number[];
  binning_x: number;
  binning_y: number;
}

export interface SensorMsgsChannelFloat32 {
  name: string;
  values: number[];
}

export interface SensorMsgsCompressedImage {
  header: StdMsgsHeader;
  format: string;
  data: number[];
}

export interface SensorMsgsFluidPressure {
  header: StdMsgsHeader;
  fluid_pressure: number;
  variance: number;
}

export interface SensorMsgsIlluminance {
  header: StdMsgsHeader;
  illuminance: number;
  variance: number;
}

export interface SensorMsgsImage {
  header: StdMsgsHeader;
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: number[];
}

export interface SensorMsgsImu {
  header: StdMsgsHeader;
  orientation: GeometryMsgsQuaternion;
  orientation_covariance: number[];
  angular_velocity: GeometryMsgsVector3;
  angular_velocity_covariance: number[];
  linear_acceleration: GeometryMsgsVector3;
  linear_acceleration_covariance: number[];
}

export interface SensorMsgsJointState {
  header: StdMsgsHeader;
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

export interface SensorMsgsJoy {
  header: StdMsgsHeader;
  axes: number[];
  buttons: number[];
}

export interface SensorMsgsJoyFeedback {
  type: number;
  id: number;
  intensity: number;
}

export enum SensorMsgsJoyFeedbackConst {
  TYPE_LED = 0,
  TYPE_RUMBLE = 1,
  TYPE_BUZZER = 2,
}

export interface SensorMsgsLaserEcho {
  echoes: number[];
}

export interface SensorMsgsLaserScan {
  header: StdMsgsHeader;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

export interface SensorMsgsMagneticField {
  header: StdMsgsHeader;
  magnetic_field: GeometryMsgsVector3;
  magnetic_field_covariance: number[];
}

export interface SensorMsgsMultiDofJointState {
  header: StdMsgsHeader;
  joint_names: string[];
  transforms: GeometryMsgsTransform[];
  twist: GeometryMsgsTwist[];
  wrench: GeometryMsgsWrench[];
}

export interface SensorMsgsMultiEchoLaserScan {
  header: StdMsgsHeader;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
}

export interface SensorMsgsNavSatFix {
  header: StdMsgsHeader;
  latitude: number;
  longitude: number;
  altitude: number;
  position_covariance: number[];
  position_covariance_type: number;
}

export enum SensorMsgsNavSatFixConst {
  COVARIANCE_TYPE_UNKNOWN = 0,
  COVARIANCE_TYPE_APPROXIMATED = 1,
  COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
  COVARIANCE_TYPE_KNOWN = 3,
}

export interface SensorMsgsNavSatStatus {
  status: number;
  service: number;
}

export enum SensorMsgsNavSatStatusConst {
  STATUS_UNKNOWN = -2,
  STATUS_NO_FIX = -1,
  STATUS_FIX = 0,
  STATUS_SBAS_FIX = 1,
  STATUS_GBAS_FIX = 2,
  SERVICE_UNKNOWN = 0,
  SERVICE_GPS = 1,
  SERVICE_GLONASS = 2,
  SERVICE_COMPASS = 4,
  SERVICE_GALILEO = 8,
}

export interface SensorMsgsPointCloud {
  header: StdMsgsHeader;
  points: GeometryMsgsPoint32[];
}

export interface SensorMsgsPointCloud2 {
  header: StdMsgsHeader;
  height: number;
  width: number;
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  data: number[];
  is_dense: boolean;
}

export interface SensorMsgsPointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

export enum SensorMsgsPointFieldConst {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8,
}

export interface SensorMsgsRange {
  header: StdMsgsHeader;
  radiation_type: number;
  field_of_view: number;
  min_range: number;
  max_range: number;
  range: number;
  variance: number;
}

export enum SensorMsgsRangeConst {
  ULTRASOUND = 0,
  INFRARED = 1,
}

export interface SensorMsgsRegionOfInterest {
  x_offset: number;
  y_offset: number;
  height: number;
  width: number;
  do_rectify: boolean;
}

export interface SensorMsgsRelativeHumidity {
  header: StdMsgsHeader;
  relative_humidity: number;
  variance: number;
}

export interface SensorMsgsSetCameraInfoRequest {
  camera_info: SensorMsgsCameraInfo;
}

export interface SensorMsgsSetCameraInfoResponse {
  success: boolean;
  status_message: string;
}

export interface SensorMsgsTemperature {
  header: StdMsgsHeader;
  temperature: number;
  variance: number;
}

export interface SensorMsgsTimeReference {
  header: StdMsgsHeader;
  time_ref: { sec: number, nanosec: number };
  source: string;
}

export interface ShapeMsgsMesh {
  vertices: GeometryMsgsPoint[];
}

export interface ShapeMsgsMeshTriangle {
  vertex_indices: number[];
}

export interface ShapeMsgsPlane {
  coef: number[];
}

export interface ShapeMsgsSolidPrimitive {
  type: number;
  dimensions: number[];
  polygon: GeometryMsgsPolygon;
}

export enum ShapeMsgsSolidPrimitiveConst {
  BOX = 1,
  SPHERE = 2,
  CYLINDER = 3,
  CONE = 4,
  PRISM = 5,
  BOX_X = 0,
  BOX_Y = 1,
  BOX_Z = 2,
  SPHERE_RADIUS = 0,
  CYLINDER_HEIGHT = 0,
  CYLINDER_RADIUS = 1,
  CONE_HEIGHT = 0,
  CONE_RADIUS = 1,
  PRISM_HEIGHT = 0,
}

export interface StatisticsMsgsMetricsMessage {
  measurement_source_name: string;
  metrics_source: string;
  unit: string;
  window_start: { sec: number, nanosec: number };
  window_stop: { sec: number, nanosec: number };
}

export interface StatisticsMsgsStatisticDataPoint {
  data_type: number;
  data: number;
}

export interface StdMsgsBool {
  data: boolean;
}

export interface StdMsgsByte {
  data: number;
}

export interface StdMsgsByteMultiArray {
  data: number[];
}

export interface StdMsgsChar {
  data: number;
}

export interface StdMsgsColorRgba {
  r: number;
  g: number;
  b: number;
  a: number;
}

export interface StdMsgsFloat32 {
  data: number;
}

export interface StdMsgsFloat32MultiArray {
  data: number[];
}

export interface StdMsgsFloat64 {
  data: number;
}

export interface StdMsgsFloat64MultiArray {
  data: number[];
}

export interface StdMsgsHeader {
  stamp: { sec: number, nanosec: number };
  frame_id: string;
}

export interface StdMsgsInt16 {
  data: number;
}

export interface StdMsgsInt16MultiArray {
  data: number[];
}

export interface StdMsgsInt32 {
  data: number;
}

export interface StdMsgsInt32MultiArray {
  data: number[];
}

export interface StdMsgsInt64 {
  data: number;
}

export interface StdMsgsInt64MultiArray {
  data: number[];
}

export interface StdMsgsInt8 {
  data: number;
}

export interface StdMsgsInt8MultiArray {
  data: number[];
}

export interface StdMsgsMultiArrayDimension {
  label: string;
  size: number;
  stride: number;
}

export interface StdMsgsMultiArrayLayout {
  data_offset: number;
}

export interface StdMsgsString {
  data: string;
}

export interface StdMsgsUInt16 {
  data: number;
}

export interface StdMsgsUInt16MultiArray {
  data: number[];
}

export interface StdMsgsUInt32 {
  data: number;
}

export interface StdMsgsUInt32MultiArray {
  data: number[];
}

export interface StdMsgsUInt64 {
  data: number;
}

export interface StdMsgsUInt64MultiArray {
  data: number[];
}

export interface StdMsgsUInt8 {
  data: number;
}

export interface StdMsgsUInt8MultiArray {
  data: number[];
}

export interface StereoMsgsDisparityImage {
  header: StdMsgsHeader;
  image: SensorMsgsImage;
  f: number;
  t: number;
  valid_window: SensorMsgsRegionOfInterest;
  min_disparity: number;
  max_disparity: number;
  delta_d: number;
}

export interface Tf2MsgsFrameGraphResponse {
  frame_yaml: string;
}

export interface Tf2MsgsLookupTransformActionGoal {
  target_frame: string;
  source_frame: string;
  source_time: { sec: number, nanosec: number };
  timeout: { sec: number, nanosec: number };
  target_time: { sec: number, nanosec: number };
  fixed_frame: string;
  advanced: boolean;
}

export interface Tf2MsgsLookupTransformActionResult {
  transform: GeometryMsgsTransformStamped;
  error: Tf2MsgsTf2Error;
}

export interface Tf2MsgsTf2Error {
  error: number;
  error_string: string;
}

export enum Tf2MsgsTf2ErrorConst {
  NO_ERROR = 0,
  LOOKUP_ERROR = 1,
  CONNECTIVITY_ERROR = 2,
  EXTRAPOLATION_ERROR = 3,
  INVALID_ARGUMENT_ERROR = 4,
  TIMEOUT_ERROR = 5,
  TRANSFORM_ERROR = 6,
}

export interface Tf2MsgsTfMessage {
  transforms: GeometryMsgsTransformStamped[];
}

export interface TrajectoryMsgsJointTrajectory {
  header: StdMsgsHeader;
  joint_names: string[];
}

export interface TrajectoryMsgsJointTrajectoryPoint {
  positions: number[];
  velocities: number[];
  accelerations: number[];
  effort: number[];
  time_from_start: { sec: number, nanosec: number };
}

export interface TrajectoryMsgsMultiDofJointTrajectory {
  header: StdMsgsHeader;
  joint_names: string[];
}

export interface TrajectoryMsgsMultiDofJointTrajectoryPoint {
  transforms: GeometryMsgsTransform[];
  velocities: GeometryMsgsTwist[];
  accelerations: GeometryMsgsTwist[];
  time_from_start: { sec: number, nanosec: number };
}

export interface UniqueIdentifierMsgsUuid {
  uuid: number[];
}

export interface VisualizationMsgsGetInteractiveMarkersResponse {
  sequence_number: number;
}

export interface VisualizationMsgsImageMarker {
  header: StdMsgsHeader;
  ns: string;
  id: number;
  type: number;
  action: number;
  position: GeometryMsgsPoint;
  scale: number;
  outline_color: StdMsgsColorRgba;
  filled: number;
  fill_color: StdMsgsColorRgba;
  lifetime: { sec: number, nanosec: number };
  points: GeometryMsgsPoint[];
  outline_colors: StdMsgsColorRgba[];
}

export enum VisualizationMsgsImageMarkerConst {
  CIRCLE = 0,
  LINE_STRIP = 1,
  LINE_LIST = 2,
  POLYGON = 3,
  POINTS = 4,
  ADD = 0,
  REMOVE = 1,
}

export interface VisualizationMsgsInteractiveMarker {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
  name: string;
  description: string;
  scale: number;
}

export interface VisualizationMsgsInteractiveMarkerControl {
  name: string;
  orientation: GeometryMsgsQuaternion;
  orientation_mode: number;
  interaction_mode: number;
  always_visible: boolean;
  independent_marker_orientation: boolean;
  description: string;
}

export enum VisualizationMsgsInteractiveMarkerControlConst {
  INHERIT = 0,
  FIXED = 1,
  VIEW_FACING = 2,
  NONE = 0,
  MENU = 1,
  BUTTON = 2,
  MOVE_AXIS = 3,
  MOVE_PLANE = 4,
  ROTATE_AXIS = 5,
  MOVE_ROTATE = 6,
  MOVE_3D = 7,
  ROTATE_3D = 8,
  MOVE_ROTATE_3D = 9,
}

export interface VisualizationMsgsInteractiveMarkerFeedback {
  header: StdMsgsHeader;
  client_id: string;
  marker_name: string;
  control_name: string;
  event_type: number;
  pose: GeometryMsgsPose;
  menu_entry_id: number;
  mouse_point: GeometryMsgsPoint;
  mouse_point_valid: boolean;
}

export enum VisualizationMsgsInteractiveMarkerFeedbackConst {
  KEEP_ALIVE = 0,
  POSE_UPDATE = 1,
  MENU_SELECT = 2,
  BUTTON_CLICK = 3,
  MOUSE_DOWN = 4,
  MOUSE_UP = 5,
}

export interface VisualizationMsgsInteractiveMarkerInit {
  server_id: string;
  seq_num: number;
}

export interface VisualizationMsgsInteractiveMarkerPose {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
  name: string;
}

export interface VisualizationMsgsInteractiveMarkerUpdate {
  server_id: string;
  seq_num: number;
  type: number;
  erases: string[];
}

export enum VisualizationMsgsInteractiveMarkerUpdateConst {
  KEEP_ALIVE = 0,
  UPDATE = 1,
}

export interface VisualizationMsgsMarker {
  header: StdMsgsHeader;
  ns: string;
  id: number;
  type: number;
  action: number;
  pose: GeometryMsgsPose;
  scale: GeometryMsgsVector3;
  color: StdMsgsColorRgba;
  lifetime: { sec: number, nanosec: number };
  frame_locked: boolean;
  points: GeometryMsgsPoint[];
  colors: StdMsgsColorRgba[];
  texture_resource: string;
  texture: SensorMsgsCompressedImage;
  text: string;
  mesh_resource: string;
  mesh_use_embedded_materials: boolean;
}

export enum VisualizationMsgsMarkerConst {
  ARROW = 0,
  CUBE = 1,
  SPHERE = 2,
  CYLINDER = 3,
  LINE_STRIP = 4,
  LINE_LIST = 5,
  CUBE_LIST = 6,
  SPHERE_LIST = 7,
  POINTS = 8,
  TEXT_VIEW_FACING = 9,
  MESH_RESOURCE = 10,
  TRIANGLE_LIST = 11,
  ARROW_STRIP = 12,
  ADD = 0,
  MODIFY = 0,
  DELETE = 2,
  DELETEALL = 3,
}

export interface VisualizationMsgsMenuEntry {
  id: number;
  parent_id: number;
  title: string;
  command: string;
  command_type: number;
}

export enum VisualizationMsgsMenuEntryConst {
  FEEDBACK = 0,
  ROSRUN = 1,
  ROSLAUNCH = 2,
}

export interface VisualizationMsgsMeshFile {
  filename: string;
  data: number[];
}

export interface VisualizationMsgsUvCoordinate {
  u: number;
  v: number;
}