# rviz_record
RViz2 panel plugin to record the RenderPanel to video via GStreamer, controlled by ROS services.

## install

This plugin was developed on ROS2 Humble. Required gstreamer packages can be installed via:

```sudo apt install gstreamer1.0 gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav qtbase5-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev```

This ROS2 package can be built in the standard way, followed by sourcing install/setup.bash.

## usage

After sourcing the package install, launch rviz2, and add an rviz_record/RecordPanel via Panels/Add New Panel. The newly added plugin is now awaiting service calls to start and stop movies.

The current version is merely capturing the screen real estate corresponding to the rviz capture window, so if the window is covered by anything, it will show up in your movie. A version which does not merely screen capture is possible via Ogre calls and is in the works.

Starting a movie can be accomplished via any ros2 client via a service call. From the command line, we have, for example:

```ros2 service call /rviz_record/start rviz_record/srv/StartRecording "{filename: '/tmp/explore7.mp4', fps: 10.0, scale: 1.0, codec: 'h264', use_sim_time: false}"```

Stopping is accomplished via:

```ros2 service call /rviz_record/stop rviz_record/srv/StopRecording "{}"```


## Credits/LLM story

This code was developed from my concept via GPT5 (OpenAI, September 2025). This progressed as follows:

- I suggested to GPT5 that it would be useful to have a tool which captured the rviz render window to a movie. We had some discussion and agreed for a first version, an rviz2 plugin (c++) gstreamer-based screen capture implementation would be sufficient.
- GPT5 implemented this as a complete ros2 package
- Numerous build issues/iterations followed.
- After building, I discovered that the movie was not capturing the screen or was empty. Multiple debugging and update rounds followed.
- GPT5s first set of implmentations could not capture the rviz render window without blanking out either the rviz window, the movie, or both.
- GPT5 oscillated between Ogre and gstreamer implementations, and building and updating code became increasingly difficult. GPT5 finally produced a version which emitted valid frames, but was poorly integrated with previous versions. Mayhem ensued. I finally guessed at which components of various versions were working and did a manual merge. This solidified into the current version.