/* eslint-disable */
// These files were generated using "ros-typescript-generator"
export interface ActionMsgsCancelGoal {
  request: ActionMsgsCancelGoalRequest;
  response: ActionMsgsCancelGoalResponse;
}

export interface ActionMsgsCancelGoalRequest {
  goal_info: ActionMsgsGoalInfo;
}

export interface ActionMsgsCancelGoalResponse {
  return_code: number;
  goals_canceling: ActionMsgsGoalInfo[];
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
  goal_info: ActionMsgsGoalInfo;
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

export interface ActionMsgsGoalStatusArray {
  status_list: ActionMsgsGoalStatus[];
}

export interface ActionTutorialsInterfacesFibonacciActionFeedback {
  sequence: number[];
}

export interface ActionTutorialsInterfacesFibonacciActionGoal {
  order: number;
}

export interface ActionTutorialsInterfacesFibonacciActionResult {
  sequence: number[];
}

export interface ActionlibMsgsGoalId {
  stamp: { sec: number, nanosec: number };
  id: string;
}

export interface ActionlibMsgsGoalStatus {
  goal_id: ActionlibMsgsGoalId;
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
  status_list: ActionlibMsgsGoalStatus[];
}

export interface ActuatorMsgsActuators {
  header: StdMsgsHeader;
  position: number[];
  velocity: number[];
  normalized: number[];
}

export interface ActuatorMsgsActuatorsAngularPosition {
  position: number[];
}

export interface ActuatorMsgsActuatorsAngularVelocity {
  velocity: number[];
}

export interface ActuatorMsgsActuatorsLinearPosition {
  position: number[];
}

export interface ActuatorMsgsActuatorsLinearVelocity {
  velocity: number[];
}

export interface ActuatorMsgsActuatorsNormalized {
  header: StdMsgsHeader;
  normalized: number[];
}

export interface ActuatorMsgsActuatorsPosition {
  header: StdMsgsHeader;
  angular: ActuatorMsgsActuatorsAngularPosition;
  linear: ActuatorMsgsActuatorsLinearPosition;
}

export interface ActuatorMsgsActuatorsVelocity {
  header: StdMsgsHeader;
  angular: ActuatorMsgsActuatorsAngularVelocity;
  linear: ActuatorMsgsActuatorsLinearVelocity;
}

export interface BuiltinInterfacesDuration {
  sec: number;
  nanosec: number;
}

export interface BuiltinInterfacesTime {
  sec: number;
  nanosec: number;
}

export interface CompositionInterfacesListNodesResponse {
  full_node_names: string[];
  unique_ids: number[];
}

export interface CompositionInterfacesLoadNode {
  request: CompositionInterfacesLoadNodeRequest;
  response: CompositionInterfacesLoadNodeResponse;
}

export interface CompositionInterfacesLoadNodeRequest {
  package_name: string;
  plugin_name: string;
  node_name: string;
  node_namespace: string;
  log_level: number;
  remap_rules: string[];
  parameters: RclInterfacesParameter[];
  extra_arguments: RclInterfacesParameter[];
}

export interface CompositionInterfacesLoadNodeResponse {
  success: boolean;
  error_message: string;
  full_node_name: string;
  unique_id: number;
}

export interface CompositionInterfacesUnloadNode {
  request: CompositionInterfacesUnloadNodeRequest;
  response: CompositionInterfacesUnloadNodeResponse;
}

export interface CompositionInterfacesUnloadNodeRequest {
  unique_id: number;
}

export interface CompositionInterfacesUnloadNodeResponse {
  success: boolean;
  error_message: string;
}

export interface DiagnosticMsgsAddDiagnostics {
  request: DiagnosticMsgsAddDiagnosticsRequest;
  response: DiagnosticMsgsAddDiagnosticsResponse;
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
  status: DiagnosticMsgsDiagnosticStatus[];
}

export interface DiagnosticMsgsDiagnosticStatus {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
  values: DiagnosticMsgsKeyValue[];
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
  status: DiagnosticMsgsDiagnosticStatus[];
}

export interface ExampleInterfacesAddTwoInts {
  request: ExampleInterfacesAddTwoIntsRequest;
  response: ExampleInterfacesAddTwoIntsResponse;
}

export interface ExampleInterfacesAddTwoIntsRequest {
  a: number;
  b: number;
}

export interface ExampleInterfacesAddTwoIntsResponse {
  sum: number;
}

export interface ExampleInterfacesBool {
  data: boolean;
}

export interface ExampleInterfacesByte {
  data: number;
}

export interface ExampleInterfacesByteMultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesChar {
  data: number;
}

export interface ExampleInterfacesFibonacciActionFeedback {
  sequence: number[];
}

export interface ExampleInterfacesFibonacciActionGoal {
  order: number;
}

export interface ExampleInterfacesFibonacciActionResult {
  sequence: number[];
}

export interface ExampleInterfacesFloat32 {
  data: number;
}

export interface ExampleInterfacesFloat32MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesFloat64 {
  data: number;
}

export interface ExampleInterfacesFloat64MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesInt16 {
  data: number;
}

export interface ExampleInterfacesInt16MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesInt32 {
  data: number;
}

export interface ExampleInterfacesInt32MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesInt64 {
  data: number;
}

export interface ExampleInterfacesInt64MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesInt8 {
  data: number;
}

export interface ExampleInterfacesInt8MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesMultiArrayDimension {
  label: string;
  size: number;
  stride: number;
}

export interface ExampleInterfacesMultiArrayLayout {
  dim: ExampleInterfacesMultiArrayDimension[];
  data_offset: number;
}

export interface ExampleInterfacesSetBool {
  request: ExampleInterfacesSetBoolRequest;
  response: ExampleInterfacesSetBoolResponse;
}

export interface ExampleInterfacesSetBoolRequest {
  data: boolean;
}

export interface ExampleInterfacesSetBoolResponse {
  success: boolean;
  message: string;
}

export interface ExampleInterfacesString {
  data: string;
}

export interface ExampleInterfacesTriggerResponse {
  success: boolean;
  message: string;
}

export interface ExampleInterfacesUInt16 {
  data: number;
}

export interface ExampleInterfacesUInt16MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesUInt32 {
  data: number;
}

export interface ExampleInterfacesUInt32MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesUInt64 {
  data: number;
}

export interface ExampleInterfacesUInt64MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesUInt8 {
  data: number;
}

export interface ExampleInterfacesUInt8MultiArray {
  layout: ExampleInterfacesMultiArrayLayout;
  data: number[];
}

export interface ExampleInterfacesWString {
  data: string;
}

export interface GeometryMsgsAccel {
  linear: GeometryMsgsVector3;
  angular: GeometryMsgsVector3;
}

export interface GeometryMsgsAccelStamped {
  header: StdMsgsHeader;
  accel: GeometryMsgsAccel;
}

export interface GeometryMsgsAccelWithCovariance {
  accel: GeometryMsgsAccel;
  covariance: number[];
}

export interface GeometryMsgsAccelWithCovarianceStamped {
  header: StdMsgsHeader;
  accel: GeometryMsgsAccelWithCovariance;
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
  inertia: GeometryMsgsInertia;
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
  point: GeometryMsgsPoint;
}

export interface GeometryMsgsPolygon {
  points: GeometryMsgsPoint32[];
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
  polygon: GeometryMsgsPolygon;
}

export interface GeometryMsgsPose {
  position: GeometryMsgsPoint;
  orientation: GeometryMsgsQuaternion;
}

export interface GeometryMsgsPose2D {
  x: number;
  y: number;
  theta: number;
}

export interface GeometryMsgsPoseArray {
  header: StdMsgsHeader;
  poses: GeometryMsgsPose[];
}

export interface GeometryMsgsPoseStamped {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
}

export interface GeometryMsgsPoseWithCovariance {
  pose: GeometryMsgsPose;
  covariance: number[];
}

export interface GeometryMsgsPoseWithCovarianceStamped {
  header: StdMsgsHeader;
  pose: GeometryMsgsPoseWithCovariance;
}

export interface GeometryMsgsQuaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface GeometryMsgsQuaternionStamped {
  header: StdMsgsHeader;
  quaternion: GeometryMsgsQuaternion;
}

export interface GeometryMsgsTransform {
  translation: GeometryMsgsVector3;
  rotation: GeometryMsgsQuaternion;
}

export interface GeometryMsgsTransformStamped {
  header: StdMsgsHeader;
  child_frame_id: string;
  transform: GeometryMsgsTransform;
}

export interface GeometryMsgsTwist {
  linear: GeometryMsgsVector3;
  angular: GeometryMsgsVector3;
}

export interface GeometryMsgsTwistStamped {
  header: StdMsgsHeader;
  twist: GeometryMsgsTwist;
}

export interface GeometryMsgsTwistWithCovariance {
  twist: GeometryMsgsTwist;
  covariance: number[];
}

export interface GeometryMsgsTwistWithCovarianceStamped {
  header: StdMsgsHeader;
  twist: GeometryMsgsTwistWithCovariance;
}

export interface GeometryMsgsVector3 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsVector3Stamped {
  header: StdMsgsHeader;
  vector: GeometryMsgsVector3;
}

export interface GeometryMsgsVelocityStamped {
  header: StdMsgsHeader;
  body_frame_id: string;
  reference_frame_id: string;
  velocity: GeometryMsgsTwist;
}

export interface GeometryMsgsWrench {
  force: GeometryMsgsVector3;
  torque: GeometryMsgsVector3;
}

export interface GeometryMsgsWrenchStamped {
  header: StdMsgsHeader;
  wrench: GeometryMsgsWrench;
}

export interface GpsMsgsGpsFix {
  header: StdMsgsHeader;
  status: GpsMsgsGpsStatus;
  latitude: number;
  longitude: number;
  altitude: number;
  track: number;
  speed: number;
  climb: number;
  pitch: number;
  roll: number;
  dip: number;
  time: number;
  gdop: number;
  pdop: number;
  hdop: number;
  vdop: number;
  tdop: number;
  err: number;
  err_horz: number;
  err_vert: number;
  err_track: number;
  err_speed: number;
  err_climb: number;
  err_time: number;
  err_pitch: number;
  err_roll: number;
  err_dip: number;
  position_covariance: number[];
  position_covariance_type: number;
}

export enum GpsMsgsGpsFixConst {
  COVARIANCE_TYPE_UNKNOWN = 0,
  COVARIANCE_TYPE_APPROXIMATED = 1,
  COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
  COVARIANCE_TYPE_KNOWN = 3,
}

export interface GpsMsgsGpsStatus {
  header: StdMsgsHeader;
  satellites_used: number;
  satellite_used_prn: number[];
  satellites_visible: number;
  satellite_visible_prn: number[];
  satellite_visible_z: number[];
  satellite_visible_azimuth: number[];
  satellite_visible_snr: number[];
  status: number;
  motion_source: number;
  orientation_source: number;
  position_source: number;
}

export enum GpsMsgsGpsStatusConst {
  STATUS_NO_FIX = -1,
  STATUS_FIX = 0,
  STATUS_SBAS_FIX = 1,
  STATUS_GBAS_FIX = 2,
  STATUS_DGPS_FIX = 18,
  STATUS_WAAS_FIX = 33,
  SOURCE_NONE = 0,
  SOURCE_GPS = 1,
  SOURCE_POINTS = 2,
  SOURCE_DOPPLER = 4,
  SOURCE_ALTIMETER = 8,
  SOURCE_MAGNETIC = 16,
  SOURCE_GYRO = 32,
  SOURCE_ACCEL = 64,
}

export interface LifecycleMsgsChangeState {
  request: LifecycleMsgsChangeStateRequest;
  response: LifecycleMsgsChangeStateResponse;
}

export interface LifecycleMsgsChangeStateRequest {
  transition: LifecycleMsgsTransition;
}

export interface LifecycleMsgsChangeStateResponse {
  success: boolean;
}

export interface LifecycleMsgsGetAvailableStatesResponse {
  available_states: LifecycleMsgsState[];
}

export interface LifecycleMsgsGetAvailableTransitionsResponse {
  available_transitions: LifecycleMsgsTransitionDescription[];
}

export interface LifecycleMsgsGetStateResponse {
  current_state: LifecycleMsgsState;
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

export interface LifecycleMsgsTransitionDescription {
  transition: LifecycleMsgsTransition;
  start_state: LifecycleMsgsState;
  goal_state: LifecycleMsgsState;
}

export interface LifecycleMsgsTransitionEvent {
  timestamp: number;
  transition: LifecycleMsgsTransition;
  start_state: LifecycleMsgsState;
  goal_state: LifecycleMsgsState;
}

export interface LoggingDemoConfigLogger {
  request: LoggingDemoConfigLoggerRequest;
  response: LoggingDemoConfigLoggerResponse;
}

export interface LoggingDemoConfigLoggerRequest {
  logger_name: string;
  level: string;
}

export interface LoggingDemoConfigLoggerResponse {
  success: boolean;
}

export interface MapMsgsGetMapRoi {
  request: MapMsgsGetMapRoiRequest;
  response: MapMsgsGetMapRoiResponse;
}

export interface MapMsgsGetMapRoiRequest {
  x: number;
  y: number;
  l_x: number;
  l_y: number;
}

export interface MapMsgsGetMapRoiResponse {
  sub_map: NavMsgsOccupancyGrid;
}

export interface MapMsgsGetPointMapResponse {
  map: SensorMsgsPointCloud2;
}

export interface MapMsgsGetPointMapRoi {
  request: MapMsgsGetPointMapRoiRequest;
  response: MapMsgsGetPointMapRoiResponse;
}

export interface MapMsgsGetPointMapRoiRequest {
  x: number;
  y: number;
  z: number;
  r: number;
  l_x: number;
  l_y: number;
  l_z: number;
}

export interface MapMsgsGetPointMapRoiResponse {
  sub_map: SensorMsgsPointCloud2;
}

export interface MapMsgsOccupancyGridUpdate {
  header: StdMsgsHeader;
  x: number;
  y: number;
  width: number;
  height: number;
  data: number[];
}

export interface MapMsgsPointCloud2Update {
  header: StdMsgsHeader;
  type: number;
  points: SensorMsgsPointCloud2;
}

export enum MapMsgsPointCloud2UpdateConst {
  ADD = 0,
  DELETE = 1,
}

export interface MapMsgsProjectedMap {
  map: NavMsgsOccupancyGrid;
  min_z: number;
  max_z: number;
}

export interface MapMsgsProjectedMapInfo {
  frame_id: string;
  x: number;
  y: number;
  width: number;
  height: number;
  min_z: number;
  max_z: number;
}

export interface MapMsgsProjectedMapsInfoRequest {
  projected_maps_info: MapMsgsProjectedMapInfo[];
}

export interface MapMsgsSaveMapRequest {
  filename: StdMsgsString;
}

export interface MapMsgsSetMapProjectionsResponse {
  projected_maps_info: MapMsgsProjectedMapInfo[];
}

export interface NavMsgsGetMapResponse {
  map: NavMsgsOccupancyGrid;
}

export interface NavMsgsGetPlan {
  request: NavMsgsGetPlanRequest;
  response: NavMsgsGetPlanResponse;
}

export interface NavMsgsGetPlanRequest {
  start: GeometryMsgsPoseStamped;
  goal: GeometryMsgsPoseStamped;
  tolerance: number;
}

export interface NavMsgsGetPlanResponse {
  plan: NavMsgsPath;
}

export interface NavMsgsGridCells {
  header: StdMsgsHeader;
  cell_width: number;
  cell_height: number;
  cells: GeometryMsgsPoint[];
}

export interface NavMsgsLoadMap {
  request: NavMsgsLoadMapRequest;
  response: NavMsgsLoadMapResponse;
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
  info: NavMsgsMapMetaData;
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

export interface NavMsgsSetMap {
  request: NavMsgsSetMapRequest;
  response: NavMsgsSetMapResponse;
}

export interface NavMsgsSetMapRequest {
  map: NavMsgsOccupancyGrid;
  initial_pose: GeometryMsgsPoseWithCovarianceStamped;
}

export interface NavMsgsSetMapResponse {
  success: boolean;
}

export interface PclMsgsModelCoefficients {
  header: StdMsgsHeader;
  values: number[];
}

export interface PclMsgsPointIndices {
  header: StdMsgsHeader;
  indices: number[];
}

export interface PclMsgsPolygonMesh {
  header: StdMsgsHeader;
  cloud: SensorMsgsPointCloud2;
  polygons: PclMsgsVertices[];
}

export interface PclMsgsUpdateFilename {
  request: PclMsgsUpdateFilenameRequest;
  response: PclMsgsUpdateFilenameResponse;
}

export interface PclMsgsUpdateFilenameRequest {
  filename: string;
}

export interface PclMsgsUpdateFilenameResponse {
  success: boolean;
}

export interface PclMsgsVertices {
  vertices: number[];
}

export interface PendulumMsgsJointCommand {
  position: number;
}

export interface PendulumMsgsJointState {
  position: number;
  velocity: number;
  effort: number;
}

export interface PendulumMsgsRttestResults {
  stamp: { sec: number, nanosec: number };
  command: PendulumMsgsJointCommand;
  state: PendulumMsgsJointState;
  cur_latency: number;
  mean_latency: number;
  min_latency: number;
  max_latency: number;
  minor_pagefaults: number;
  major_pagefaults: number;
}

export enum RclInterfacesParameterTypeConst {
  PARAMETER_NOT_SET = 0,
  PARAMETER_BOOL = 1,
  PARAMETER_INTEGER = 2,
  PARAMETER_DOUBLE = 3,
  PARAMETER_STRING = 4,
  PARAMETER_BYTE_ARRAY = 5,
  PARAMETER_BOOL_ARRAY = 6,
  PARAMETER_INTEGER_ARRAY = 7,
  PARAMETER_DOUBLE_ARRAY = 8,
  PARAMETER_STRING_ARRAY = 9,
}

export interface RclInterfacesDescribeParameters {
  request: RclInterfacesDescribeParametersRequest;
  response: RclInterfacesDescribeParametersResponse;
}

export interface RclInterfacesDescribeParametersRequest {
  names: string[];
}

export interface RclInterfacesDescribeParametersResponse {
  descriptors: RclInterfacesParameterDescriptor[];
}

export interface RclInterfacesFloatingPointRange {
  from_value: number;
  to_value: number;
  step: number;
}

export interface RclInterfacesGetLoggerLevels {
  request: RclInterfacesGetLoggerLevelsRequest;
  response: RclInterfacesGetLoggerLevelsResponse;
}

export interface RclInterfacesGetLoggerLevelsRequest {
  names: string[];
}

export interface RclInterfacesGetLoggerLevelsResponse {
  levels: RclInterfacesLoggerLevel[];
}

export interface RclInterfacesGetParameterTypes {
  request: RclInterfacesGetParameterTypesRequest;
  response: RclInterfacesGetParameterTypesResponse;
}

export interface RclInterfacesGetParameterTypesRequest {
  names: string[];
}

export interface RclInterfacesGetParameterTypesResponse {
  types: number[];
}

export interface RclInterfacesGetParameters {
  request: RclInterfacesGetParametersRequest;
  response: RclInterfacesGetParametersResponse;
}

export interface RclInterfacesGetParametersRequest {
  names: string[];
}

export interface RclInterfacesGetParametersResponse {
  values: RclInterfacesParameterValue[];
}

export interface RclInterfacesIntegerRange {
  from_value: number;
  to_value: number;
  step: number;
}

export interface RclInterfacesListParameters {
  request: RclInterfacesListParametersRequest;
  response: RclInterfacesListParametersResponse;
}

export interface RclInterfacesListParametersRequest {
  prefixes: string[];
  depth: number;
}

export enum RclInterfacesListParametersRequestConst {
  DEPTH_RECURSIVE = 0,
}

export interface RclInterfacesListParametersResponse {
  result: RclInterfacesListParametersResult;
}

export interface RclInterfacesListParametersResult {
  names: string[];
  prefixes: string[];
}

export interface RclInterfacesLog {
  stamp: { sec: number, nanosec: number };
  level: number;
  name: string;
  msg: string;
  file: string;
  function: string;
  line: number;
}

export enum RclInterfacesLogConst {
  DEBUG = 10,
  INFO = 20,
  WARN = 30,
  ERROR = 40,
  FATAL = 50,
}

export interface RclInterfacesLoggerLevel {
  name: string;
  level: number;
}

export enum RclInterfacesLoggerLevelConst {
  LOG_LEVEL_UNKNOWN = 0,
  LOG_LEVEL_DEBUG = 10,
  LOG_LEVEL_INFO = 20,
  LOG_LEVEL_WARN = 30,
  LOG_LEVEL_ERROR = 40,
  LOG_LEVEL_FATAL = 50,
}

export interface RclInterfacesParameter {
  name: string;
  value: RclInterfacesParameterValue;
}

export interface RclInterfacesParameterDescriptor {
  name: string;
  type: number;
  description: string;
  additional_constraints: string;
  read_only: boolean;
  dynamic_typing: boolean;
  floating_point_range: RclInterfacesFloatingPointRange[];
  integer_range: RclInterfacesIntegerRange[];
}

export interface RclInterfacesParameterEvent {
  stamp: { sec: number, nanosec: number };
  node: string;
  new_parameters: RclInterfacesParameter[];
  changed_parameters: RclInterfacesParameter[];
  deleted_parameters: RclInterfacesParameter[];
}

export interface RclInterfacesParameterEventDescriptors {
  new_parameters: RclInterfacesParameterDescriptor[];
  changed_parameters: RclInterfacesParameterDescriptor[];
  deleted_parameters: RclInterfacesParameterDescriptor[];
}

export interface RclInterfacesParameterValue {
  type: number;
  bool_value: boolean;
  integer_value: number;
  double_value: number;
  string_value: string;
  byte_array_value: number[];
  bool_array_value: boolean[];
  integer_array_value: number[];
  double_array_value: number[];
  string_array_value: string[];
}

export interface RclInterfacesSetLoggerLevels {
  request: RclInterfacesSetLoggerLevelsRequest;
  response: RclInterfacesSetLoggerLevelsResponse;
}

export interface RclInterfacesSetLoggerLevelsRequest {
  levels: RclInterfacesLoggerLevel[];
}

export interface RclInterfacesSetLoggerLevelsResponse {
  results: RclInterfacesSetLoggerLevelsResult[];
}

export interface RclInterfacesSetLoggerLevelsResult {
  successful: boolean;
  reason: string;
}

export interface RclInterfacesSetParameters {
  request: RclInterfacesSetParametersRequest;
  response: RclInterfacesSetParametersResponse;
}

export interface RclInterfacesSetParametersAtomically {
  request: RclInterfacesSetParametersAtomicallyRequest;
  response: RclInterfacesSetParametersAtomicallyResponse;
}

export interface RclInterfacesSetParametersAtomicallyRequest {
  parameters: RclInterfacesParameter[];
}

export interface RclInterfacesSetParametersAtomicallyResponse {
  result: RclInterfacesSetParametersResult;
}

export interface RclInterfacesSetParametersRequest {
  parameters: RclInterfacesParameter[];
}

export interface RclInterfacesSetParametersResponse {
  results: RclInterfacesSetParametersResult[];
}

export interface RclInterfacesSetParametersResult {
  successful: boolean;
  reason: string;
}

export interface RmwDdsCommonGid {
  data: number[];
}

export interface RmwDdsCommonNodeEntitiesInfo {
  node_namespace: string;
  node_name: string;
  reader_gid_seq: RmwDdsCommonGid[];
  writer_gid_seq: RmwDdsCommonGid[];
}

export interface RmwDdsCommonParticipantEntitiesInfo {
  gid: RmwDdsCommonGid;
  node_entities_info_seq: RmwDdsCommonNodeEntitiesInfo[];
}

export interface RosGzInterfacesAltimeter {
  header: StdMsgsHeader;
  vertical_position: number;
  vertical_velocity: number;
  vertical_reference: number;
}

export interface RosGzInterfacesContact {
  collision1: RosGzInterfacesEntity;
  collision2: RosGzInterfacesEntity;
  positions: GeometryMsgsVector3[];
  normals: GeometryMsgsVector3[];
  depths: number[];
  wrenches: RosGzInterfacesJointWrench[];
}

export interface RosGzInterfacesContacts {
  header: StdMsgsHeader;
  contacts: RosGzInterfacesContact[];
}

export interface RosGzInterfacesControlWorld {
  request: RosGzInterfacesControlWorldRequest;
  response: RosGzInterfacesControlWorldResponse;
}

export interface RosGzInterfacesControlWorldRequest {
  world_control: RosGzInterfacesWorldControl;
}

export interface RosGzInterfacesControlWorldResponse {
  success: boolean;
}

export interface RosGzInterfacesDataframe {
  header: StdMsgsHeader;
  src_address: string;
  dst_address: string;
  data: number[];
  rssi: number;
}

export interface RosGzInterfacesDeleteEntity {
  request: RosGzInterfacesDeleteEntityRequest;
  response: RosGzInterfacesDeleteEntityResponse;
}

export interface RosGzInterfacesDeleteEntityRequest {
  entity: RosGzInterfacesEntity;
}

export interface RosGzInterfacesDeleteEntityResponse {
  success: boolean;
}

export interface RosGzInterfacesEntity {
  id: number;
  name: string;
  type: number;
}

export enum RosGzInterfacesEntityConst {
  NONE = 0,
  LIGHT = 1,
  MODEL = 2,
  LINK = 3,
  VISUAL = 4,
  COLLISION = 5,
  SENSOR = 6,
  JOINT = 7,
}

export interface RosGzInterfacesEntityFactory {
  name: string;
  allow_renaming: boolean;
  sdf: string;
  sdf_filename: string;
  clone_name: string;
  pose: GeometryMsgsPose;
  relative_to: string;
}

export interface RosGzInterfacesEntityWrench {
  header: StdMsgsHeader;
  entity: RosGzInterfacesEntity;
  wrench: GeometryMsgsWrench;
}

export interface RosGzInterfacesFloat32Array {
  data: number[];
}

export interface RosGzInterfacesGuiCamera {
  header: StdMsgsHeader;
  name: string;
  view_controller: string;
  pose: GeometryMsgsPose;
  track: RosGzInterfacesTrackVisual;
  projection_type: string;
}

export interface RosGzInterfacesJointWrench {
  header: StdMsgsHeader;
  body_1_name: StdMsgsString;
  body_1_id: StdMsgsUInt32;
  body_2_name: StdMsgsString;
  body_2_id: StdMsgsUInt32;
  body_1_wrench: GeometryMsgsWrench;
  body_2_wrench: GeometryMsgsWrench;
}

export interface RosGzInterfacesLight {
  header: StdMsgsHeader;
  name: string;
  type: number;
  pose: GeometryMsgsPose;
  diffuse: StdMsgsColorRgba;
  specular: StdMsgsColorRgba;
  attenuation_constant: number;
  attenuation_linear: number;
  attenuation_quadratic: number;
  direction: GeometryMsgsVector3;
  range: number;
  cast_shadows: boolean;
  spot_inner_angle: number;
  spot_outer_angle: number;
  spot_falloff: number;
  id: number;
  parent_id: number;
  intensity: number;
}

export enum RosGzInterfacesLightConst {
  POINT = 0,
  SPOT = 1,
  DIRECTIONAL = 2,
}

export interface RosGzInterfacesMaterialColor {
  header: StdMsgsHeader;
  entity: RosGzInterfacesEntity;
  ambient: StdMsgsColorRgba;
  diffuse: StdMsgsColorRgba;
  specular: StdMsgsColorRgba;
  emissive: StdMsgsColorRgba;
  shininess: number;
  entity_match: number;
}

export enum RosGzInterfacesMaterialColorConst {
  FIRST = 0,
  ALL = 1,
}

export interface RosGzInterfacesParamVec {
  header: StdMsgsHeader;
  params: RclInterfacesParameter[];
}

export interface RosGzInterfacesSensorNoise {
  header: StdMsgsHeader;
  type: number;
  mean: number;
  stddev: number;
  bias_mean: number;
  bias_stddev: number;
  precision: number;
  dynamic_bias_stddev: number;
  dynamic_bias_correlation_time: number;
}

export enum RosGzInterfacesSensorNoiseConst {
  NONE = 0,
  GAUSSIAN = 2,
  GAUSSIAN_QUANTIZED = 3,
}

export interface RosGzInterfacesSetEntityPose {
  request: RosGzInterfacesSetEntityPoseRequest;
  response: RosGzInterfacesSetEntityPoseResponse;
}

export interface RosGzInterfacesSetEntityPoseRequest {
  entity: RosGzInterfacesEntity;
  pose: GeometryMsgsPose;
}

export interface RosGzInterfacesSetEntityPoseResponse {
  success: boolean;
}

export interface RosGzInterfacesSpawnEntity {
  request: RosGzInterfacesSpawnEntityRequest;
  response: RosGzInterfacesSpawnEntityResponse;
}

export interface RosGzInterfacesSpawnEntityRequest {
  entity_factory: RosGzInterfacesEntityFactory;
}

export interface RosGzInterfacesSpawnEntityResponse {
  success: boolean;
}

export interface RosGzInterfacesStringVec {
  header: StdMsgsHeader;
  data: string[];
}

export interface RosGzInterfacesTrackVisual {
  header: StdMsgsHeader;
  name: string;
  id: number;
  inherit_orientation: boolean;
  min_dist: number;
  max_dist: number;
  is_static: boolean;
  use_model_frame: boolean;
  xyz: GeometryMsgsVector3;
  inherit_yaw: boolean;
}

export interface RosGzInterfacesVideoRecord {
  header: StdMsgsHeader;
  start: boolean;
  stop: boolean;
  format: string;
  save_filename: string;
}

export interface RosGzInterfacesWorldControl {
  pause: boolean;
  step: boolean;
  multi_step: number;
  reset: RosGzInterfacesWorldReset;
  seed: number;
  run_to_sim_time: { sec: number, nanosec: number };
}

export interface RosGzInterfacesWorldReset {
  all: boolean;
  time_only: boolean;
  model_only: boolean;
}

export interface Rosbag2InterfacesBurst {
  request: Rosbag2InterfacesBurstRequest;
  response: Rosbag2InterfacesBurstResponse;
}

export interface Rosbag2InterfacesBurstRequest {
  num_messages: number;
}

export interface Rosbag2InterfacesBurstResponse {
  actually_burst: number;
}

export interface Rosbag2InterfacesGetRateResponse {
  rate: number;
}

export interface Rosbag2InterfacesIsPausedResponse {
  paused: boolean;
}

export interface Rosbag2InterfacesPlay {
  request: Rosbag2InterfacesPlayRequest;
  response: Rosbag2InterfacesPlayResponse;
}

export interface Rosbag2InterfacesPlayNextResponse {
  success: boolean;
}

export interface Rosbag2InterfacesPlayRequest {
  start_offset: { sec: number, nanosec: number };
  playback_duration: { sec: number, nanosec: number };
  playback_until_timestamp: { sec: number, nanosec: number };
}

export interface Rosbag2InterfacesPlayResponse {
  success: boolean;
}

export interface Rosbag2InterfacesReadSplitEvent {
  closed_file: string;
  opened_file: string;
  node_name: string;
}

export interface Rosbag2InterfacesSeek {
  request: Rosbag2InterfacesSeekRequest;
  response: Rosbag2InterfacesSeekResponse;
}

export interface Rosbag2InterfacesSeekRequest {
  time: { sec: number, nanosec: number };
}

export interface Rosbag2InterfacesSeekResponse {
  success: boolean;
}

export interface Rosbag2InterfacesSetRate {
  request: Rosbag2InterfacesSetRateRequest;
  response: Rosbag2InterfacesSetRateResponse;
}

export interface Rosbag2InterfacesSetRateRequest {
  rate: number;
}

export interface Rosbag2InterfacesSetRateResponse {
  success: boolean;
}

export interface Rosbag2InterfacesSnapshotResponse {
  success: boolean;
}

export interface Rosbag2InterfacesWriteSplitEvent {
  closed_file: string;
  opened_file: string;
  node_name: string;
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
  roi: SensorMsgsRegionOfInterest;
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

export interface SensorMsgsJoyFeedbackArray {
  array: SensorMsgsJoyFeedback[];
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
  ranges: SensorMsgsLaserEcho[];
  intensities: SensorMsgsLaserEcho[];
}

export interface SensorMsgsNavSatFix {
  header: StdMsgsHeader;
  status: SensorMsgsNavSatStatus;
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
  channels: SensorMsgsChannelFloat32[];
}

export interface SensorMsgsPointCloud2 {
  header: StdMsgsHeader;
  height: number;
  width: number;
  fields: SensorMsgsPointField[];
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

export interface SensorMsgsSetCameraInfo {
  request: SensorMsgsSetCameraInfoRequest;
  response: SensorMsgsSetCameraInfoResponse;
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

export interface ServiceMsgsServiceEventInfo {
  event_type: number;
  stamp: { sec: number, nanosec: number };
  client_gid: number[];
  sequence_number: number;
}

export enum ServiceMsgsServiceEventInfoConst {
  REQUEST_SENT = 0,
  REQUEST_RECEIVED = 1,
  RESPONSE_SENT = 2,
  RESPONSE_RECEIVED = 3,
}

export interface ShapeMsgsMesh {
  triangles: ShapeMsgsMeshTriangle[];
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

export enum StatisticsMsgsStatisticDataTypeConst {
  STATISTICS_DATA_TYPE_UNINITIALIZED = 0,
  STATISTICS_DATA_TYPE_AVERAGE = 1,
  STATISTICS_DATA_TYPE_MINIMUM = 2,
  STATISTICS_DATA_TYPE_MAXIMUM = 3,
  STATISTICS_DATA_TYPE_STDDEV = 4,
  STATISTICS_DATA_TYPE_SAMPLE_COUNT = 5,
}

export interface StatisticsMsgsMetricsMessage {
  measurement_source_name: string;
  metrics_source: string;
  unit: string;
  window_start: { sec: number, nanosec: number };
  window_stop: { sec: number, nanosec: number };
  statistics: StatisticsMsgsStatisticDataPoint[];
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
  layout: StdMsgsMultiArrayLayout;
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
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsFloat64 {
  data: number;
}

export interface StdMsgsFloat64MultiArray {
  layout: StdMsgsMultiArrayLayout;
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
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt32 {
  data: number;
}

export interface StdMsgsInt32MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt64 {
  data: number;
}

export interface StdMsgsInt64MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt8 {
  data: number;
}

export interface StdMsgsInt8MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsMultiArrayDimension {
  label: string;
  size: number;
  stride: number;
}

export interface StdMsgsMultiArrayLayout {
  dim: StdMsgsMultiArrayDimension[];
  data_offset: number;
}

export interface StdMsgsString {
  data: string;
}

export interface StdMsgsUInt16 {
  data: number;
}

export interface StdMsgsUInt16MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt32 {
  data: number;
}

export interface StdMsgsUInt32MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt64 {
  data: number;
}

export interface StdMsgsUInt64MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt8 {
  data: number;
}

export interface StdMsgsUInt8MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdSrvsSetBool {
  request: StdSrvsSetBoolRequest;
  response: StdSrvsSetBoolResponse;
}

export interface StdSrvsSetBoolRequest {
  data: boolean;
}

export interface StdSrvsSetBoolResponse {
  success: boolean;
  message: string;
}

export interface StdSrvsTriggerResponse {
  success: boolean;
  message: string;
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

export interface TheoraImageTransportPacket {
  header: StdMsgsHeader;
  data: number[];
  b_o_s: number;
  e_o_s: number;
  granulepos: number;
  packetno: number;
}

export interface TrajectoryMsgsJointTrajectory {
  header: StdMsgsHeader;
  joint_names: string[];
  points: TrajectoryMsgsJointTrajectoryPoint[];
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
  points: TrajectoryMsgsMultiDofJointTrajectoryPoint[];
}

export interface TrajectoryMsgsMultiDofJointTrajectoryPoint {
  transforms: GeometryMsgsTransform[];
  velocities: GeometryMsgsTwist[];
  accelerations: GeometryMsgsTwist[];
  time_from_start: { sec: number, nanosec: number };
}

export interface TurtlesimColor {
  r: number;
  g: number;
  b: number;
}

export interface TurtlesimKillRequest {
  name: string;
}

export interface TurtlesimPose {
  x: number;
  y: number;
  theta: number;
  linear_velocity: number;
  angular_velocity: number;
}

export interface TurtlesimRotateAbsoluteActionFeedback {
  remaining: number;
}

export interface TurtlesimRotateAbsoluteActionGoal {
  theta: number;
}

export interface TurtlesimRotateAbsoluteActionResult {
  delta: number;
}

export interface TurtlesimSetPenRequest {
  r: number;
  g: number;
  b: number;
  width: number;
  off: number;
}

export interface TurtlesimSpawn {
  request: TurtlesimSpawnRequest;
  response: TurtlesimSpawnResponse;
}

export interface TurtlesimSpawnRequest {
  x: number;
  y: number;
  theta: number;
  name: string;
}

export interface TurtlesimSpawnResponse {
  name: string;
}

export interface TurtlesimTeleportAbsoluteRequest {
  x: number;
  y: number;
  theta: number;
}

export interface TurtlesimTeleportRelativeRequest {
  linear: number;
  angular: number;
}

export interface TypeDescriptionInterfacesField {
  name: string;
  type: TypeDescriptionInterfacesFieldType;
  default_value: string;
}

export interface TypeDescriptionInterfacesFieldType {
  type_id: number;
  capacity: number;
  string_capacity: number;
  nested_type_name: string;
}

export enum TypeDescriptionInterfacesFieldTypeConst {
  FIELD_TYPE_NOT_SET = 0,
  FIELD_TYPE_NESTED_TYPE = 1,
  FIELD_TYPE_INT8 = 2,
  FIELD_TYPE_UINT8 = 3,
  FIELD_TYPE_INT16 = 4,
  FIELD_TYPE_UINT16 = 5,
  FIELD_TYPE_INT32 = 6,
  FIELD_TYPE_UINT32 = 7,
  FIELD_TYPE_INT64 = 8,
  FIELD_TYPE_UINT64 = 9,
  FIELD_TYPE_FLOAT = 10,
  FIELD_TYPE_DOUBLE = 11,
  FIELD_TYPE_LONG_DOUBLE = 12,
  FIELD_TYPE_CHAR = 13,
  FIELD_TYPE_WCHAR = 14,
  FIELD_TYPE_BOOLEAN = 15,
  FIELD_TYPE_BYTE = 16,
  FIELD_TYPE_STRING = 17,
  FIELD_TYPE_WSTRING = 18,
  FIELD_TYPE_FIXED_STRING = 19,
  FIELD_TYPE_FIXED_WSTRING = 20,
  FIELD_TYPE_BOUNDED_STRING = 21,
  FIELD_TYPE_BOUNDED_WSTRING = 22,
  FIELD_TYPE_NESTED_TYPE_ARRAY = 49,
  FIELD_TYPE_INT8_ARRAY = 50,
  FIELD_TYPE_UINT8_ARRAY = 51,
  FIELD_TYPE_INT16_ARRAY = 52,
  FIELD_TYPE_UINT16_ARRAY = 53,
  FIELD_TYPE_INT32_ARRAY = 54,
  FIELD_TYPE_UINT32_ARRAY = 55,
  FIELD_TYPE_INT64_ARRAY = 56,
  FIELD_TYPE_UINT64_ARRAY = 57,
  FIELD_TYPE_FLOAT_ARRAY = 58,
  FIELD_TYPE_DOUBLE_ARRAY = 59,
  FIELD_TYPE_LONG_DOUBLE_ARRAY = 60,
  FIELD_TYPE_CHAR_ARRAY = 61,
  FIELD_TYPE_WCHAR_ARRAY = 62,
  FIELD_TYPE_BOOLEAN_ARRAY = 63,
  FIELD_TYPE_BYTE_ARRAY = 64,
  FIELD_TYPE_STRING_ARRAY = 65,
  FIELD_TYPE_WSTRING_ARRAY = 66,
  FIELD_TYPE_FIXED_STRING_ARRAY = 67,
  FIELD_TYPE_FIXED_WSTRING_ARRAY = 68,
  FIELD_TYPE_BOUNDED_STRING_ARRAY = 69,
  FIELD_TYPE_BOUNDED_WSTRING_ARRAY = 70,
  FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE = 97,
  FIELD_TYPE_INT8_BOUNDED_SEQUENCE = 98,
  FIELD_TYPE_UINT8_BOUNDED_SEQUENCE = 99,
  FIELD_TYPE_INT16_BOUNDED_SEQUENCE = 100,
  FIELD_TYPE_UINT16_BOUNDED_SEQUENCE = 101,
  FIELD_TYPE_INT32_BOUNDED_SEQUENCE = 102,
  FIELD_TYPE_UINT32_BOUNDED_SEQUENCE = 103,
  FIELD_TYPE_INT64_BOUNDED_SEQUENCE = 104,
  FIELD_TYPE_UINT64_BOUNDED_SEQUENCE = 105,
  FIELD_TYPE_FLOAT_BOUNDED_SEQUENCE = 106,
  FIELD_TYPE_DOUBLE_BOUNDED_SEQUENCE = 107,
  FIELD_TYPE_LONG_DOUBLE_BOUNDED_SEQUENCE = 108,
  FIELD_TYPE_CHAR_BOUNDED_SEQUENCE = 109,
  FIELD_TYPE_WCHAR_BOUNDED_SEQUENCE = 110,
  FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE = 111,
  FIELD_TYPE_BYTE_BOUNDED_SEQUENCE = 112,
  FIELD_TYPE_STRING_BOUNDED_SEQUENCE = 113,
  FIELD_TYPE_WSTRING_BOUNDED_SEQUENCE = 114,
  FIELD_TYPE_FIXED_STRING_BOUNDED_SEQUENCE = 115,
  FIELD_TYPE_FIXED_WSTRING_BOUNDED_SEQUENCE = 116,
  FIELD_TYPE_BOUNDED_STRING_BOUNDED_SEQUENCE = 117,
  FIELD_TYPE_BOUNDED_WSTRING_BOUNDED_SEQUENCE = 118,
  FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE = 145,
  FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE = 146,
  FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE = 147,
  FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE = 148,
  FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE = 149,
  FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE = 150,
  FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE = 151,
  FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE = 152,
  FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE = 153,
  FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE = 154,
  FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE = 155,
  FIELD_TYPE_LONG_DOUBLE_UNBOUNDED_SEQUENCE = 156,
  FIELD_TYPE_CHAR_UNBOUNDED_SEQUENCE = 157,
  FIELD_TYPE_WCHAR_UNBOUNDED_SEQUENCE = 158,
  FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE = 159,
  FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE = 160,
  FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE = 161,
  FIELD_TYPE_WSTRING_UNBOUNDED_SEQUENCE = 162,
  FIELD_TYPE_FIXED_STRING_UNBOUNDED_SEQUENCE = 163,
  FIELD_TYPE_FIXED_WSTRING_UNBOUNDED_SEQUENCE = 164,
  FIELD_TYPE_BOUNDED_STRING_UNBOUNDED_SEQUENCE = 165,
  FIELD_TYPE_BOUNDED_WSTRING_UNBOUNDED_SEQUENCE = 166,
}

export interface TypeDescriptionInterfacesGetTypeDescription {
  request: TypeDescriptionInterfacesGetTypeDescriptionRequest;
  response: TypeDescriptionInterfacesGetTypeDescriptionResponse;
}

export interface TypeDescriptionInterfacesGetTypeDescriptionRequest {
  type_name: string;
  type_hash: string;
  include_type_sources: boolean;
}

export interface TypeDescriptionInterfacesGetTypeDescriptionResponse {
  successful: boolean;
  failure_reason: string;
  type_description: TypeDescriptionInterfacesTypeDescription;
  type_sources: TypeDescriptionInterfacesTypeSource[];
  extra_information: TypeDescriptionInterfacesKeyValue[];
}

export interface TypeDescriptionInterfacesIndividualTypeDescription {
  type_name: string;
  fields: TypeDescriptionInterfacesField[];
}

export interface TypeDescriptionInterfacesKeyValue {
  key: string;
  value: string;
}

export interface TypeDescriptionInterfacesTypeDescription {
  type_description: TypeDescriptionInterfacesIndividualTypeDescription;
  referenced_type_descriptions: TypeDescriptionInterfacesIndividualTypeDescription[];
}

export interface TypeDescriptionInterfacesTypeSource {
  type_name: string;
  encoding: string;
  raw_file_contents: string;
}

export interface UniqueIdentifierMsgsUuid {
  uuid: number[];
}

export interface VisionMsgsBoundingBox2D {
  center: VisionMsgsPose2D;
  size_x: number;
  size_y: number;
}

export interface VisionMsgsBoundingBox2DArray {
  header: StdMsgsHeader;
  boxes: VisionMsgsBoundingBox2D[];
}

export interface VisionMsgsBoundingBox3D {
  center: GeometryMsgsPose;
  size: GeometryMsgsVector3;
}

export interface VisionMsgsBoundingBox3DArray {
  header: StdMsgsHeader;
  boxes: VisionMsgsBoundingBox3D[];
}

export interface VisionMsgsClassification {
  header: StdMsgsHeader;
  results: VisionMsgsObjectHypothesis[];
}

export interface VisionMsgsDetection2D {
  header: StdMsgsHeader;
  results: VisionMsgsObjectHypothesisWithPose[];
  bbox: VisionMsgsBoundingBox2D;
  id: string;
}

export interface VisionMsgsDetection2DArray {
  header: StdMsgsHeader;
  detections: VisionMsgsDetection2D[];
}

export interface VisionMsgsDetection3D {
  header: StdMsgsHeader;
  results: VisionMsgsObjectHypothesisWithPose[];
  bbox: VisionMsgsBoundingBox3D;
  id: string;
}

export interface VisionMsgsDetection3DArray {
  header: StdMsgsHeader;
  detections: VisionMsgsDetection3D[];
}

export interface VisionMsgsLabelInfo {
  header: StdMsgsHeader;
  class_map: VisionMsgsVisionClass[];
  threshold: number;
}

export interface VisionMsgsObjectHypothesis {
  class_id: string;
  score: number;
}

export interface VisionMsgsObjectHypothesisWithPose {
  hypothesis: VisionMsgsObjectHypothesis;
  pose: GeometryMsgsPoseWithCovariance;
}

export interface VisionMsgsPoint2D {
  x: number;
  y: number;
}

export interface VisionMsgsPose2D {
  position: VisionMsgsPoint2D;
  theta: number;
}

export interface VisionMsgsVisionClass {
  class_id: number;
  class_name: string;
}

export interface VisionMsgsVisionInfo {
  header: StdMsgsHeader;
  method: string;
  database_location: string;
  database_version: number;
}

export interface VisualizationMsgsGetInteractiveMarkersResponse {
  sequence_number: number;
  markers: VisualizationMsgsInteractiveMarker[];
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
  menu_entries: VisualizationMsgsMenuEntry[];
  controls: VisualizationMsgsInteractiveMarkerControl[];
}

export interface VisualizationMsgsInteractiveMarkerControl {
  name: string;
  orientation: GeometryMsgsQuaternion;
  orientation_mode: number;
  interaction_mode: number;
  always_visible: boolean;
  markers: VisualizationMsgsMarker[];
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
  markers: VisualizationMsgsInteractiveMarker[];
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
  markers: VisualizationMsgsInteractiveMarker[];
  poses: VisualizationMsgsInteractiveMarkerPose[];
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
  uv_coordinates: VisualizationMsgsUvCoordinate[];
  text: string;
  mesh_resource: string;
  mesh_file: VisualizationMsgsMeshFile;
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

export interface VisualizationMsgsMarkerArray {
  markers: VisualizationMsgsMarker[];
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