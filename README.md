# AutoFRC

This is a study into creating a fully automatic FRC robot. 

This process relies on several key algorithms:
- pose estiation: using the WpiLib DifferentialDrivePoseEstimatorClass
- global position updates: I am using an algorithm called UcoSLAM that takes a camera feed and outputs position in 3d space
- pathing planning: Informed Random Rapidly Expanding Tree *
- Traveling Saleman searches to find the fastest way to collect multiple games objects
- HSV thresholding to detected yellow balls

Future potential features:
- robot detection (either ML or detect color of bumpers)
- intelligent defences
- avoiding moving obstancles while path planning
  - kalman filters