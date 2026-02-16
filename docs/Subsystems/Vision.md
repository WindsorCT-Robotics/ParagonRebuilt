# Vision Subsystem

An algorithim that calculates and returns the pose estimation by having visual recongiztion and converts it into data to identify targets for the drive station.

## Dependencies

Requires a camera that can accurately read april tags.

## Public APIs

### Methods

#### getPoseEstimate

Calls the LimeLlight's position estimate to estimate the robot's position based off april tags and returns an error type incase of a bad postion estimate.

#### getPoseEstimateSupplier

Gets a stream of 'PoseEstimates'. 