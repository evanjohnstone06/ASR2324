#!/bin/sh
cd tflite1
surce tflite1-env/bin/activate
sudo pigpiod
sudo python lidarRot.py & python3 evanTest.py --modeldir=Sample_TFLite_model
