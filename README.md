# vo
A vo system refer to orbslam

## TODO list
writing class frame, it need keyframe and convert mappoint and so on.
- [ ] ORB
  - [x] ORBextractor Finished but FPS only has 15. Need more test
  - [ ] ORBmatcher
  - [x] ORBVocbulary
- [ ] Frame
  - [ ] FrameDrawer
- [ ] KeyFrame
  - [ ] KeyFrameDatabase
- [ ] Map
  - [ ] MapDrawer
  - [ ] MapPoint
- [ ] Optimizer
  - [ ] Sim3Solver
  - [ ] PnPsolver
  - [ ] Converter
- [ ] System
- [ ] Tracking
- [ ] LocalMapping
- [ ] LoopClosing
- [ ] Viewer

## Unit test
ORBextractor's unit test read image from d435i, extract keypoint, compute descriptor and show keypooint.

The FPS is only 15, need to verify is some bug in it.