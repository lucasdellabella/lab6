# Lucas Della Bella
# Saqib Ali

''' Take a picture

Takes most recent image from Cozmo
Converts it to 8-bit black and white
Saves to destination
'''

import sys
import cozmo
import datetime
import time
import numpy as np
from sklearn import metrics
from classifier import ImageClassifier
from collections import deque, defaultdict
import marker_detection
import skimage
import pdb
import pickle

def run(sdk_conn):
    
    robot = sdk_conn.wait_for_robot()
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()

    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    robot.set_lift_height(height=0, duration=0.5).wait_for_completed()

    idleState(robot)

def orderState(robot):
    robot.drive_wheels(240, 125.6, duration=5.5)

def inspectionState(robot):
    def turn():
        turn_in_place_action = robot.turn_in_place(angle=cozmo.util.Angle(degrees=90)).wait_for_completed()  # ? in parallel

    def drive_side(robot, task):
        set_lift_action = robot.set_lift_height(height=task, duration=2.5, in_parallel=True)
        drive_straight_action = robot.drive_straight(distance=cozmo.util.Distance(distance_mm=200), speed=cozmo.util.Speed(80), in_parallel=True).wait_for_completed()

    LOWER = 0
    RAISE = 1
    for i in range(2):
        drive_side(robot, RAISE)
        turn()
        drive_side(robot, LOWER)
        turn()

def droneState(robot):
    def pickup_putdown(evt, image_box=None, obj=None, pose=None, updated=None):
        print("TESTING")
        print(obj)
        robot.pickup_object(obj).wait_for_completed()
        robot.drive_straight(distance=cozmo.util.Distance(distance_mm=100),
                             speed=cozmo.util.Speed(80)).wait_for_completed()
        robot.place_object_on_ground_here(obj).wait_for_completed()
        robot.drive_straight(distance=cozmo.util.Distance(distance_mm=-100),
                             speed=cozmo.util.Speed(80)).wait_for_completed()

    look_around = robot.start_behavior(cozmo.behavior._BehaviorType(name='LookAroundInPlace', id=5))
    objectObservedEvt = robot.wait_for(cozmo.objects.EvtObjectObserved, timeout=30)
    look_around.stop()
    pickup_putdown(objectObservedEvt, obj=objectObservedEvt.obj)

def idleState(robot):
    def find_majority(d):
        dict = defaultdict(lambda: 0)
        for item in d:
            dict[item] += 1
        curr_max = 0
        max_item = ""
        for item in d:
            if dict[item] > curr_max:
                curr_max = dict[item]
                max_item = item
        return max_item

    classifier = ImageClassifier()

    (train_raw, train_labels) = classifier.load_data_from_folder('./photos/')#load_sorted_data('./photos')
    (test_raw, test_labels) = classifier.load_data_from_folder('./train/')

    # convert images into features
    train_data = np.array(classifier.extract_image_features(train_raw))
    test_data = np.array(classifier.extract_image_features(test_raw))

    print('entering train classifier')
    # train model and test on training data
    classifier.train_classifier(train_data, train_labels)
    predicted_labels_for_train = classifier.predict_labels(train_data)
    train_accuracy = metrics.accuracy_score(train_labels, predicted_labels_for_train)

    print('entering test classifier')

    predicted_labels_for_test = classifier.predict_labels(test_data)
    test_accuracy = metrics.accuracy_score(test_labels, predicted_labels_for_test)
    #if test_accuracy >= 1.0:
    print("Training results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels_for_train))
    print("Accuracy: ", train_accuracy)
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels_for_train, average='micro'))

    # test model
    print("\Testing results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels_for_test))
    print ("Test Labels: ", test_labels)
    print("Accuracy: ", test_accuracy)
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels_for_test, average='micro'))
    print("\n")
    if test_accuracy == 1.0:
        print('''
        ||~~~~~~~~~~~~~~~~~~
        ||~~~~~~~~~~~~~~~~~~
        ||~~~~~~VICTORY~~~~~
        ||~~~~~~~~~~~~~~~~~~
        ||~~~~~~~~~~~~~~~~~~
        ||
        ||
        ||
        ||
        ||
                ''')


    d = deque()

    # Start main program
    while(True):
        # have cozmo understand images
        latest_image = robot.world.latest_image
        #new_image = np.array(classifier.extract_image_features(np.array([np.array(latest_image.raw_image)])))
        raw_image = np.array(robot.world.wait_for(cozmo.world.EvtNewCameraImage).image.raw_image)
        raw_image = skimage.color.rgb2gray(raw_image)

        marker = marker_detection.detect_marker(raw_image)

        # Check if a marker was detected
        if marker['detected']:
            # Get the cropped, unwarped image of just the marker
            marker_image = marker['unwarped_image']
            #marker_image = marker_image.reshape(240, 320, 1)
            marker_image = np.array(classifier.extract_image_features(np.array([np.array(marker_image)])))
            # Use the marker image for improved classification, e.g.:
            # # marker_type = my_classification_function(marker_image)
            observation = classifier.predict_labels(marker_image)[0]
            d.append(observation)
            if len(d) >= 6:
                d.popleft()
                majority = find_majority(d)
                if majority != 'none':
                    print('' + majority + ' detected!')
                    print(d)
                    robot.say_text(majority).wait_for_completed()
                    d = deque()
                if majority == 'drone': droneState(robot)
                if majority == 'inspection': inspectionState(robot)
                if majority == 'order': orderState(robot)
        else:
            d.append('none')
            if len(d) >= 6:
                d.popleft()

if __name__ == '__main__':
    cozmo.setup_basic_logging()

    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
