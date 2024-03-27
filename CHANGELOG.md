# Changelog

All notable changes to this project will be documented in this file.

## [0.6.0] - 2024-03-27

### 🚀 Features

- Allo image_size to be used as a parameter
- Add a params_file argument

### 🐛 Bug Fixes

- Bring back delted code
- Add missing type to sender parameter

### 🚜 Refactor

- Remove debug print

### ⚙️ Miscellaneous Tasks

- Add changelog and cliff config file
- Bump version

## [0.4.0] - 2024-03-27

### 🚀 Features

- Include the header in image msg
- Add python typing support

### 🐛 Bug Fixes

- Call the right function to flip back right

### 📚 Documentation

- State that it is a ros2 wrapper

### ⚙️ Miscellaneous Tasks

- Bump version

## [0.1.3] - 2023-11-17

### 🚀 Features

- *(config)* Add parameters for imu/odom
- *(driver)* Publish odometry, imu and tf between odom and drone
- *(driver)* Create ros msgs serializers
- Add method return types
- Add docstrings to class methods
- *(config)* Add new parameters
- *(node)* Include the new parameters added in 'params.yaml'
- Add a namespace launch argument
- *(driver)* Add a safety feature to the cmd_vel

### 🐛 Bug Fixes

- *(build)* Fix the command to install tellopy from source
- Update command to install tellopy
- Fix version to follow the standards
- Correct imports and publish the pose of the drone
- *(driver)* Set the altitude limit on every takeoff
- Update topic names to not include / in the begining
- Send position from drone in the tf

### 🚜 Refactor

- Make the attributes private
- Remove debug prints
- Remove TODO comment
- Change default tello_ssid to ""

### 📚 Documentation

- Update readme to have the update subs/pubs + parameters

### ⚙️ Miscellaneous Tasks

- Update message name to FlightStats.msg
- Update the package.xml to include a description, license and version
- Bump version to 0.1.1
- Bump version to 0.1.2
- Ignore colcon generated directories
- Fix version of packages
- Add MIT license
- Fix license file name

### Build

- Add script to install tellopy from source

