from pms import Dataset, Solution, triangulate
import os
from pms_py.utils import project_root


def main():
    data_path = os.path.join(project_root(), "data")
    dataset = Dataset(data_path)
    solution = Solution(dataset)
    triangulate(solution, dataset)
