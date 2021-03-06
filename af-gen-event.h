// This file is generated by Simplicity Studio.  Please do not edit manually.
//
//

// Enclosing macro to prevent multiple inclusion
#ifndef __AF_GEN_EVENT__
#define __AF_GEN_EVENT__


// Code used to configure the cluster event mechanism
#define EMBER_AF_GENERATED_EVENT_CODE \
  EmberEventControl emberAfPollControlClusterServerTickCallbackControl1; \
  EmberEventControl emberAfIasZoneClusterServerTickCallbackControl1; \
  extern EmberEventControl ReportMessgeToMCUEventControl; \
  extern EmberEventControl SentZoneStatusEventControl; \
  extern EmberEventControl StartLowLevelEventControl; \
  extern EmberEventControl StopLowLevelEventControl; \
  extern EmberEventControl afterFiveSecondsCloseLedEventControl; \
  extern EmberEventControl emberAfKeepConnectEventControl; \
  extern EmberEventControl emberAfLedJoinNetworkStatusEventControl; \
  extern EmberEventControl emberAfPluginGpioSensorDebounceEventControl; \
  extern EmberEventControl emberAfPluginGpioSensorInterruptEventControl; \
  extern EmberEventControl emberAfPluginIasZoneServerManageQueueEventControl; \
  extern EmberEventControl emberAfPluginNetworkSteeringFinishSteeringEventControl; \
  extern EmberEventControl emberAfPluginUpdateTcLinkKeyBeginTcLinkKeyUpdateEventControl; \
  extern void ReportMessgeToMCUEventHandler(void); \
  extern void SentZoneStatusEventHandler(void); \
  extern void StartLowLevelEventHandler(void); \
  extern void StopLowLevelEventHandler(void); \
  extern void afterFiveSecondsCloseLedEventHandler(void); \
  extern void emberAfKeepConnectEventControlHandler(void); \
  extern void emberAfLedJoinNetworkStatusEventHandler(void); \
  extern void emberAfPluginGpioSensorDebounceEventHandler(void); \
  extern void emberAfPluginGpioSensorInterruptEventHandler(void); \
  extern void emberAfPluginIasZoneServerManageQueueEventHandler(void); \
  extern void emberAfPluginNetworkSteeringFinishSteeringEventHandler(void); \
  extern void emberAfPluginUpdateTcLinkKeyBeginTcLinkKeyUpdateEventHandler(void); \
  static void networkEventWrapper(EmberEventControl *control, EmberAfNetworkEventHandler handler, uint8_t networkIndex) \
  { \
    emberAfPushNetworkIndex(networkIndex); \
    emberEventControlSetInactive(*control); \
    (*handler)(); \
    emberAfPopNetworkIndex(); \
  } \
  EmberEventControl emberAfPluginEndDeviceSupportMoveNetworkEventControls[1]; \
  extern void emberAfPluginEndDeviceSupportMoveNetworkEventHandler(void); \
  void emberAfPluginEndDeviceSupportMoveNetworkEventWrapper0(void) { networkEventWrapper(&emberAfPluginEndDeviceSupportMoveNetworkEventControls[0], emberAfPluginEndDeviceSupportMoveNetworkEventHandler, 0); } \
  EmberEventControl emberAfPluginEndDeviceSupportPollingNetworkEventControls[1]; \
  extern void emberAfPluginEndDeviceSupportPollingNetworkEventHandler(void); \
  void emberAfPluginEndDeviceSupportPollingNetworkEventWrapper0(void) { networkEventWrapper(&emberAfPluginEndDeviceSupportPollingNetworkEventControls[0], emberAfPluginEndDeviceSupportPollingNetworkEventHandler, 0); } \
  EmberEventControl emberAfPluginScanDispatchScanNetworkEventControls[1]; \
  extern void emberAfPluginScanDispatchScanNetworkEventHandler(void); \
  void emberAfPluginScanDispatchScanNetworkEventWrapper0(void) { networkEventWrapper(&emberAfPluginScanDispatchScanNetworkEventControls[0], emberAfPluginScanDispatchScanNetworkEventHandler, 0); } \
  static void clusterTickWrapper(EmberEventControl *control, EmberAfTickFunction callback, uint8_t endpoint) \
  { \
    emberAfPushEndpointNetworkIndex(endpoint); \
    emberEventControlSetInactive(*control); \
    (*callback)(endpoint); \
    emberAfPopNetworkIndex(); \
  } \
  void emberAfPollControlClusterServerTickCallbackWrapperFunction1(void) { clusterTickWrapper(&emberAfPollControlClusterServerTickCallbackControl1, emberAfPollControlClusterServerTickCallback, 1); } \
  void emberAfIasZoneClusterServerTickCallbackWrapperFunction1(void) { clusterTickWrapper(&emberAfIasZoneClusterServerTickCallbackControl1, emberAfIasZoneClusterServerTickCallback, 1); } \
  EmberEventControl emberAfPluginPollControlServerCheckInEndpointEventControls[1]; \
  extern void emberAfPluginPollControlServerCheckInEndpointEventHandler(uint8_t endpoint); \
  void emberAfPluginPollControlServerCheckInEndpointEventWrapper1(void) { clusterTickWrapper(&emberAfPluginPollControlServerCheckInEndpointEventControls[0], emberAfPluginPollControlServerCheckInEndpointEventHandler, 1); } \


// EmberEventData structs used to populate the EmberEventData table
#define EMBER_AF_GENERATED_EVENTS   \
  { &emberAfPollControlClusterServerTickCallbackControl1, emberAfPollControlClusterServerTickCallbackWrapperFunction1 }, \
  { &emberAfIasZoneClusterServerTickCallbackControl1, emberAfIasZoneClusterServerTickCallbackWrapperFunction1 }, \
  { &ReportMessgeToMCUEventControl, ReportMessgeToMCUEventHandler }, \
  { &SentZoneStatusEventControl, SentZoneStatusEventHandler }, \
  { &StartLowLevelEventControl, StartLowLevelEventHandler }, \
  { &StopLowLevelEventControl, StopLowLevelEventHandler }, \
  { &afterFiveSecondsCloseLedEventControl, afterFiveSecondsCloseLedEventHandler }, \
  { &emberAfKeepConnectEventControl, emberAfKeepConnectEventControlHandler }, \
  { &emberAfLedJoinNetworkStatusEventControl, emberAfLedJoinNetworkStatusEventHandler }, \
  { &emberAfPluginGpioSensorDebounceEventControl, emberAfPluginGpioSensorDebounceEventHandler }, \
  { &emberAfPluginGpioSensorInterruptEventControl, emberAfPluginGpioSensorInterruptEventHandler }, \
  { &emberAfPluginIasZoneServerManageQueueEventControl, emberAfPluginIasZoneServerManageQueueEventHandler }, \
  { &emberAfPluginNetworkSteeringFinishSteeringEventControl, emberAfPluginNetworkSteeringFinishSteeringEventHandler }, \
  { &emberAfPluginUpdateTcLinkKeyBeginTcLinkKeyUpdateEventControl, emberAfPluginUpdateTcLinkKeyBeginTcLinkKeyUpdateEventHandler }, \
  { &emberAfPluginEndDeviceSupportMoveNetworkEventControls[0], emberAfPluginEndDeviceSupportMoveNetworkEventWrapper0 }, \
  { &emberAfPluginEndDeviceSupportPollingNetworkEventControls[0], emberAfPluginEndDeviceSupportPollingNetworkEventWrapper0 }, \
  { &emberAfPluginScanDispatchScanNetworkEventControls[0], emberAfPluginScanDispatchScanNetworkEventWrapper0 }, \
  { &emberAfPluginPollControlServerCheckInEndpointEventControls[0], emberAfPluginPollControlServerCheckInEndpointEventWrapper1 }, \


#define EMBER_AF_GENERATED_EVENT_STRINGS   \
  "Poll Control Cluster Server EP 1",  \
  "IAS Zone Cluster Server EP 1",  \
  "Report messge to m c u event control",  \
  "Sent zone status event control",  \
  "Start low level event control",  \
  "Stop low level event control",  \
  "Event data",  \
  "Ember af keep connect event control",  \
  "Ember af led join network status event control",  \
  "GPIO Sensor Interface Plugin Debounce",  \
  "GPIO Sensor Interface Plugin Interrupt",  \
  "IAS Zone Server Plugin ManageQueue",  \
  "Network Steering Plugin FinishSteering",  \
  "Update TC Link Key Plugin BeginTcLinkKeyUpdate",  \
  "End Device Support Plugin Move NWK 0", \
  "End Device Support Plugin Polling NWK 0", \
  "Scan Dispatch Plugin Scan NWK 0", \
  "Poll Control Server Cluster Plugin CheckIn EP 1", \


// The length of the event context table used to track and retrieve cluster events
#define EMBER_AF_EVENT_CONTEXT_LENGTH 2

// EmberAfEventContext structs used to populate the EmberAfEventContext table
#define EMBER_AF_GENERATED_EVENT_CONTEXT { 0x1, 0x20, false, EMBER_AF_LONG_POLL, EMBER_AF_OK_TO_SLEEP, &emberAfPollControlClusterServerTickCallbackControl1}, \
{ 0x1, 0x500, false, EMBER_AF_LONG_POLL, EMBER_AF_OK_TO_SLEEP, &emberAfIasZoneClusterServerTickCallbackControl1}


#endif // __AF_GEN_EVENT__
