#!/bin/bash
pathDatasetTUM_VI='/home/dh/ros1_orbslam/ws/dataset-corridor1_512_16' #'/Datasets/TUM_VI' #Example, it is necesary to change it by the dataset path


# # Single Session Example

# echo "Launching Magistrale 1 with Stereo-Inertial sensor"
# ./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-magistrale1_512.txt dataset-magistrale1_512_stereoi
# echo "------------------------------------"
# echo "Evaluation of Magistrale 1 trajectory with Stereo-Inertial sensor"
# python evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/magistrale1_512_16/mav0/mocap0/data.csv f_dataset-magistrale1_512_stereoi.txt --plot magistrale1_512_stereoi.pdf


# Single Session Example

echo "Launching Dataset Corridor1 512 with Stereo-Inertial sensor"
./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM-VI.yaml "$pathDatasetTUM_VI"/mav0/cam0/data "$pathDatasetTUM_VI"/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-corridor1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-corridor1_512.txt dataset-eval_corridor1_512_stereoi
echo "------------------------------------"
echo "Evaluation of Magistrale 1 trajectory with Stereo-Inertial sensor"
python evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/mav0/mocap0/data.csv /home/dh/ros1_orbslam/ws/ORB_SLAM3/f_dataset-eval_corridor1_512_stereoi.txt --plot f_dataset-corridor1_512_stereoi.pdf