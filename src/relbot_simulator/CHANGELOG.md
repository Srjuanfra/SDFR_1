# Changelog
## relbot_sim

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)

## [2025.1] - 2026-02-02

Update to make the relbot_sim work with the RELbot adapter. Also integrating of relbot2turtlesim package into one more streamlined interface.

### Added
- Added functionality originally from relbot2turtlesim
- Added limitation parameter on visualisation update speed, to match with turtlesim output by default. 

### Changed
- Changed changelog format from `.rst` to `.md` for better readability (and Git integration)

## [2024.0.1] - 2025-03-14

Some small fixes to make the system work on the real RELbot, which runs Humble instead of Jazzy.

### Added
- Add Preprocessor directive to load right `cv_bridge` header.

### Changed
- Updated Readme to have Markdown styling

### Removed
- Removed old launch file which wasn't discussed anywhere, so nobody knew about it.

## [2024.0] - 2025-02-02
Update to Jazzy, fixes some issues.

### Added
- Added `relbot_pose` output topic to communicate where the RELbot is in space
- CI/CD builder for auto-packaging without .git history

### Changed
- Updated package.xml to have dependencies notated
- Changed `cv_bridge` include from `.h` to `.hpp`
- Removed build folder and local VS Code files
- Updated model from First order integration to 20Sim generated model
- Updated Readme in-line with example
- Reduced logger-outputs to be less spammy
- Changed Twist to be consistent between RELbot and sim behaviour



## [1.1] - 2024-04-02

Fixes some file structure and ROS package issues

### Changed
- Changed name from `RELbot_simulator` to `relbot_simulator` to prevent ROS build warning
- re-named folder from `ros2_nodes` to `relbot_simulator`


## [1.0] - 2024-03-05

First version used for ASDfR. Got all working components

### Added
- Can receive input for image, separate motor controls, and Twists
- Simple first order model which can calculate where the simulated RELBOT moves
- Image manipulation based on the position of the RELbot in world space
