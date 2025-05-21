from pms import Dataset
import os
from pms_py.utils import project_root

def main():
    data_path = os.path.join(project_root(), "data")
    dataset = Dataset(data_path)
    print(dataset.camera)
    print(dataset.trajectory[0])
    print(dataset.world[0])
