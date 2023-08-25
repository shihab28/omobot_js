
"use strict";

let axis_state = require('./axis_state.js');
let gnss_fix = require('./gnss_fix.js');
let accelerometer = require('./accelerometer.js');
let proximity = require('./proximity.js');
let gnss_position = require('./gnss_position.js');
let covariance = require('./covariance.js');
let analog_voltage = require('./analog_voltage.js');
let gnss_track = require('./gnss_track.js');
let gyroscope = require('./gyroscope.js');
let temperature = require('./temperature.js');
let time_reference = require('./time_reference.js');
let magnetometer = require('./magnetometer.js');

module.exports = {
  axis_state: axis_state,
  gnss_fix: gnss_fix,
  accelerometer: accelerometer,
  proximity: proximity,
  gnss_position: gnss_position,
  covariance: covariance,
  analog_voltage: analog_voltage,
  gnss_track: gnss_track,
  gyroscope: gyroscope,
  temperature: temperature,
  time_reference: time_reference,
  magnetometer: magnetometer,
};
