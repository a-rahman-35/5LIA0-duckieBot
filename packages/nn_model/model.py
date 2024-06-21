import os
from typing import Tuple
 
import numpy as np
 
import torch
 
from dt_data_api import DataClient
# from .integration_activity import DT_TOKEN
 
from dt_device_utils import DeviceHardwareBrand, get_device_hardware_brand
 
from .constants import IMAGE_SIZE
 
JETSON_FP16 = True
 
 
def run(input, exception_on_failure=False):
    print(input)
    try:
        import subprocess
 
        program_output = subprocess.check_output(
            f"{input}", shell=True, universal_newlines=True, stderr=subprocess.STDOUT
        )
    except Exception as e:
        if exception_on_failure:
            print(e.output)
            raise e
        program_output = e.output
    print(program_output)
    return program_output.strip()
 
class Wrapper:
    def __init__(self):
        model_name = "yolov5n_blocks"
 
        # models_path = os.path.join(ASSETS_DIR, "weights")
        # dcss_models_path = "courses/mooc/objdet/data/nn_models/"
 
        # dcss_weight_file_path = os.path.join(dcss_models_path, f"{model_name}.pt")
        models_path = "/code/catkin_ws/src/5LIA0-duckieBot/packages/nn_model/weights/"
        weight_file_path = os.path.join(models_path, f"{model_name}.pt")
        
        # load pytorch model
        self.model = Model(weight_file_path)
 
    def predict(self, image: np.ndarray) -> Tuple[list, list, list]:
        return self.model.infer(image)
 
 
class Model:
    def __init__(self, weight_file_path: str):
        super().__init__()
       
        # model = torch.hub.load("/yolov5", "custom", path=weight_file_path, source="local")
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=weight_file_path, force_reload=True)
        # model = torch.load(weight_file_path)
        model.eval()
 
        use_fp16: bool = JETSON_FP16 and get_device_hardware_brand() == DeviceHardwareBrand.JETSON_NANO
 
        if use_fp16:
            model = model.half()
 
        if torch.cuda.is_available():
            self.model = model.cuda()
        else:
            self.model = model.cpu()
 
        del model
 
    def infer(self, image: np.ndarray) -> Tuple[list, list, list]:
        det = self.model(image, size=IMAGE_SIZE)
 
        xyxy = det.xyxy[0]  # grabs det of first image (aka the only image we sent to the net)
 
        if xyxy.shape[0] > 0:
            conf = xyxy[:, -2]
            clas = xyxy[:, -1]
            xyxy = xyxy[:, :-2]
 
            return xyxy.tolist(), clas.tolist(), conf.tolist()
        return [], [], []
 