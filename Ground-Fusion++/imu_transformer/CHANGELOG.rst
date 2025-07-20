^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imu_transformer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2022-11-22)
------------------
* imu_transformer: Fix transformation of the orientation of IMU.
  The previous computation was wrong. According to REP-145, IMU orientation should express attitude of the sensor frame in a world frame. imu_transformer changes the sensor frame, so it should just recompute the new attitude by transforming the old sensor frame into the new one.
* Contributors: Martin Pecka

0.3.0 (2020-06-04)
------------------
* Bump CMake version to avoid CMP0048 warning.
* Updated to package.xml format 2.
* Contributors: Tony Baltovski

0.2.3 (2019-01-30)
------------------
* update to use non deprecated pluginlib macro
* Updated maintainers.
* Contributors: Mikael Arguedas, Tony Baltovski

0.2.2 (2017-01-25)
------------------
* imu_transformer: Fix build with CATKIN_ENABLE_TESTIING=OFF
* Contributors: Alexis Ballier

0.2.1 (2015-02-04)
------------------
* Fix nodelet install
* Contributors: Paul Bovbel

0.2.0 (2015-01-30)
------------------
* Add imu_transformer node/nodelet combination using tf2
* Contributors: Paul Bovbel

0.1.3 (2014-03-30)
------------------

0.1.2 (2013-08-21)
------------------

0.1.1 (2013-07-29)
------------------
