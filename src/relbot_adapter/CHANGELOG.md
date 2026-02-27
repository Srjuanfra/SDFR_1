# Changelog
## relbot_adapter

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)

## [2025.1] - [03-02-2026]

Package Renamed from `relbot_interface` to `relbot_adapter`.

### Added
- Added CHANGELOG.md
- Added Throttle parameter to reduce output spam

### Changed
- Renamed files
- Changed name in CMakeLists.txt and package.xml
- Fixed behaviour of maximum speed if only one of the two inputs is set.


## [2024.5] - 2025-04-17

Initial working version. Should let users be able to switch between `sim` and `real`. The latter only works when the right files exist on the system.

### Added
- Input handling for separate motor values, or a Twist.
- Output handling for `relbot_sim` and a real RELbot.
- Preprocessor directive to check if xrf2_msgs exists.
- Limiting output based on parameters.




