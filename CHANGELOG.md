# Changelog

## [Unreleased]
### Changed
- plot_post_submission_log.py now plots multiple log files side by side instead of
  merging them.  This is useful for comparing different robots.
  The option to merge multiple log files has been dropped but the merging can easily be
  done beforehand, e.g. using `cat` and saving to a temporary file.

### Fixed
- Update demo_data_logging to changed interface of the RobotLogger class.
- Update configuration for `single_finger_test` to work with current software
  version.

## [1.0.0] - 2022-06-28
### Added
- Demo for forward/inverse kinematics computation for TriFingerPro.
- `trifinger_post_submission`: Measure object detection noise and trigger a
  failure if it exceeds some thresholds.
- `trifinger_post_submission`: New option `--log` to write results of the tests
  to the given log file (e.g. to monitor the object detection noise over time).

### Changed
- Make `trifinger_backend` more generic, so it can be used with TriFinger robots
  other than the "Pro" version.
- Set model for object tracking in `trifinger_backend` and
  `trifinger_robot_backend`.  This is needed due to changes in
  trifinger_object_tracking.
- Make `trifinger_post_submission` more flexible by using to separate arguments
  `--object=<object_type>` to specify the object type and `--reset` to indicate
  if the reset procedure for the given object type should be executed or not.

### Fixed
- Fix several flake8 and mypy warnings.


## [0.3.0] - 2021-08-04

There is no changelog for this or earlier versions.


[Unreleased]: https://github.com/open-dynamic-robot-initiative/robot_fingers/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/open-dynamic-robot-initiative/robot_fingers/compare/v0.3.0...v1.0.0
[0.3.0]: https://github.com/open-dynamic-robot-initiative/robot_fingers/releases/tag/v0.3.0
