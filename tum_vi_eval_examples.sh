#!/bin/bash

##uncomment for corridor 1 dataset
pathDatasetTUM_VI='/home/dh/ros1_orbslam/ws/dataset-corridor1_512_16' #'/Datasets/TUM_VI' #Example, it is necesary to change it by the dataset path

##uncomment for issac sim EuRoC dataset
# pathDatasetTUM_VI='/home/dh/ros1_orbslam/ws/shrinked_png_rawtime_noduplicates_invis'

# # Single Session Example

# echo "Launching Magistrale 1 with Stereo-Inertial sensor"
# ./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-magistrale1_512.txt dataset-magistrale1_512_stereoi
# echo "------------------------------------"
# echo "Evaluation of Magistrale 1 trajectory with Stereo-Inertial sensor"
# python evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/magistrale1_512_16/mav0/mocap0/data.csv f_dataset-magistrale1_512_stereoi.txt --plot magistrale1_512_stereoi.pdf



########testing corridor 1 dataset on stereo-inertial tum vi
# echo "Launching Dataset Corridor1 512 with Stereo-Inertial sensor"
# ./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM-VI.yaml "$pathDatasetTUM_VI"/mav0/cam0/data "$pathDatasetTUM_VI"/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-corridor1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-corridor1_512.txt dataset-eval_corridor1_512_stereoi
# echo "------------------------------------"
# echo "Evaluation of Magistrale 1 trajectory with Stereo-Inertial sensor"
# python evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/mav0/mocap0/data.csv /home/dh/ros1_orbslam/ws/ORB_SLAM3/f_dataset-eval_corridor1_512_stereoi.txt --plot f_dataset-corridor1_512_stereoi.pdf



########testing corridor 1 dataset on stereo tum vi with TUM-VI camera settings
# echo "Launching Dataset Corridor1 512 with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM-VI.yaml "$pathDatasetTUM_VI"/mav0/cam0/data "$pathDatasetTUM_VI"/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-corridor1_512.txt dataset-eval_corridor1_512_stereo


# echo "------------------------------------"
# # echo "Evaluation of Dataset Corridor1 512 trajectory with Stereosensor"
# # python evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/mav0/mocap0/data.csv /home/dh/ros1_orbslam/ws/ORB_SLAM3/f_dataset-eval_corridor1_512_stereo.txt --plot f_dataset-corridor1_512_stereo.pdf


# # Monocular Examples
##############testing datasets with Monocular sensor stereo only with mono_tum_vi cpp file
# echo "Launching Issac Sim EuRoC with Monocular sensor stereo only with mono_tum_vi cpp file"
# ./Examples/Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC_shrinked_fisheye.yaml "$pathDatasetTUM_VI"/mav0/cam0/data /home/dh/ros1_orbslam/ws/timestamp_shrinked_png_rawtime_noduplicates_invis.txt dataset-test_issacsim
# echo "Evaluation of png_rawtime_invis trajectory with Monocular sensor"
# python evaluation/evaluate_ate_scale.py f_dataset-test_issacsim.txt f_dataset-test_issacsim.txt --plot tum_vi_mono_issacsim.pdf --verbose 


echo "Launching Corridor 1 with Monocular sensor"
./Examples/Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM-VI.yaml "$pathDatasetTUM_VI"/mav0/cam0/data Examples/Monocular/TUM_TimeStamps/dataset-corridor1_512.txt dataset-corridor1_512_mono




###########testing issac sim dataset on stereo euroc with euroc fisheye camera settings
# echo "Launching Dataset Issac Sim EuRoC with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC_shrinked_fisheye.yaml "$pathDatasetTUM_VI"/mav0/cam0/data "$pathDatasetTUM_VI"/mav0/cam1/data /home/dh/ros1_orbslam/ws/timestamp_shrinked_png_rawtime_noduplicates_invis.txt dataet-test_issacsim

# # ./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC_shrinked_fisheye.yaml "$pathDatasetTUM_VI" /home/dh/ros1_orbslam/ws/timestamp_shrinked_png_rawtime_noduplicates_invis.txt dataset-test_issacsim




########testing corridor 1 dataset on stereo tum vi with TUM-VI_fisheye camera settings
# echo "Launching Dataset Corridor1 512 with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM-VI_fisheye.yaml "$pathDatasetTUM_VI"/mav0/cam0/data "$pathDatasetTUM_VI"/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-corridor1_512.txt dataset-eval_corridor1_512_stereo
# echo "------------------------------------"
# # echo "Evaluation of Dataset Corridor1 512 trajectory with Stereosensor"
# # python evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/mav0/mocap0/data.csv /home/dh/ros1_orbslam/ws/ORB_SLAM3/f_dataset-eval_corridor1_512_stereo.txt --plot f_dataset-corridor1_512_stereo.pdf


#########taken from tum_vi_examples.sh under stereo examples
# echo "Launching Room 1 with Stereo sensor"
# ./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM-VI.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-room1_512.txt dataset-room1_512_stereo



# # Monocular Examples
#############testing corridor 1 dataset on monocular visual only tum vi with TUM-VI camera settings
# echo "Launching Room 1 with Monocular sensor"
# ./Examples/Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM-VI.yaml "$pathDatasetTUM_VI"/mav0/cam0/data Examples/Monocular/TUM_TimeStamps/dataset-room1_512.txt dataset-room1_512_mono
