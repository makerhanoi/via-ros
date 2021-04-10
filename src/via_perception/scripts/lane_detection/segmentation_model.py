import os
import cv2
import numpy as np
import onnxruntime as rt


class LaneLineSegmentationModel:

    def __init__(self, model_path, use_gpu=False):
        self.model = rt.InferenceSession(model_path)

    def predict(self, origin_img):
        """Lane line segmentation from bgr image

        Args:
            origin_img: Original image

        Returns:
            np.array: Lane line mask. Value for each pixel 0->255.
                See lane_segmentation/model_output in Visualizer
        """

        net_input = cv2.resize(origin_img, (144, 144))
        net_input = (net_input[..., ::-1].astype(np.float32)) / 255.0
        net_input = np.reshape(net_input, (1, 144, 144, 3))

        ort_inputs = {self.model.get_inputs()[0].name: net_input}
        ort_outs = self.model.run(None, ort_inputs)
        mask = ort_outs[0][0][:, :, 2]
        mask = cv2.resize(mask, (origin_img.shape[1], origin_img.shape[0]))

        return mask
