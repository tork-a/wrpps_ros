^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wrpps_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Remove unnecessary catkin_python_setup and setup.py
  See https://github.com/jsk-ros-pkg/jsk_recognition/pull/2851
* Add missing dependency
* Enable to manage multiple sensors
* intensity_prox_calibrator -> intensity_model_acquisition
* Print node name in intensity_prox_calibrator for the case when multiple intensity_prox_calibrator are launched
* Add nodes converting simplified msg from Arduino into formal msg
* Prevent invalid memory access in frame_id setting
* Limit tof_period to avoid waitRangeComplete error
* Limit frame_id length to keep loop Hz
* Stabilize intensity
* Enable to disable sensors
* Generalize intensity_prox_calibrator: ToF sensor -> range sensor
* Publish NaN when ToF measurement is erroneous
* Enable to change static message fields of ToF range via rosparam
* Publish range_combined only when ToF data is recent because ToF quickly stops after target is suddenly lost
* Publish range_combined only when ToF data is recent or intensity is valid
* Initial commit
* Contributors: Shun Hasegawa
