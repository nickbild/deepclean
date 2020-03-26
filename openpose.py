import cv2
import numpy as np
import pyopenpose as op


if __name__ == '__main__':
    params = dict()
    params["model_folder"] = "/home/nick/software/openpose/openpose/models/"

    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    dst = cv2.imread("pi.jpg")

    datum = op.Datum()
    datum.cvInputData = dst
    opWrapper.emplaceAndPop([datum])
    newImage = datum.cvOutputData[:, :, :]
    cv2.imwrite("result.jpg", newImage)

