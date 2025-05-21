#include "pms/dataset.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <regex>      

namespace pms {

namespace fs = std::filesystem;

// Helper function to load trajectory without measurements
static std::vector<TrajPoint> _load_trajectory_without_meas(const std::string& filepath) {
    std::vector<TrajPoint> trajectory;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("File not found: " + filepath);
    }

    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        std::stringstream ss(line);
        std::vector<std::string> parts;
        std::string part;
        while (ss >> part) {
            parts.push_back(part);
        }

        if (parts.size() == 7) {
            try {
                int traj_id = std::stoi(parts[0]);
                Scalar odom_x = std::stod(parts[1]);
                Scalar odom_y = std::stod(parts[2]);
                Scalar odom_theta = std::stod(parts[3]);
                Scalar gt_x = std::stod(parts[4]);
                Scalar gt_y = std::stod(parts[5]);
                Scalar gt_theta = std::stod(parts[6]);

                Pose2 odometry(odom_theta, Vector2(odom_x, odom_y) );
                Pose2 ground_truth(gt_theta, Vector2(gt_x, gt_y));

                trajectory.emplace_back(traj_id, odometry, ground_truth);
            } catch (const std::exception& e) {
                std::cerr << "Skipping line " << line_num << " due to parsing error: " << line << " - " << e.what() << std::endl;
            }
        } else if (!line.empty()) { // Allow empty lines
             std::cerr << "Invalid line format in " << filepath << " at line " << line_num << ": " << line << std::endl;
        }
    }
    return trajectory;
}

// Helper function to add measurements to trajectory
static void _add_measurements(std::vector<TrajPoint>& trajectory, const std::string& meas_filepath) {
    std::ifstream file(meas_filepath);
    if (!file.is_open()) {
        throw std::runtime_error("File not found: " + meas_filepath);
    }

    std::string line;
    int seq_number = -1;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<std::string> parts;
        std::string part;
        while (ss >> part) {
            parts.push_back(part);
        }

        if (parts.size() == 2 && parts[0] == "seq:" && seq_number == -1) {
            try {
                seq_number = std::stoi(parts[1]);
            } catch (const std::exception& e) {
                std::cerr << "Skipping seq line in " << meas_filepath << " due to parsing error: " << line << " - " << e.what() << std::endl;
            }
        } else if (parts.size() == 5 && parts[0] == "point") {
            if (seq_number == -1) {
                 std::cerr << "Error in " << meas_filepath << ": Point data found before sequence number." << std::endl;
                 continue; // Or throw
            }
            try {
                int current_id = std::stoi(parts[1]);
                int actual_id = std::stoi(parts[2]);
                Scalar image_point_x = std::stod(parts[3]);
                Scalar image_point_y = std::stod(parts[4]);

                Vector2 image_point(image_point_x, image_point_y);
                Measurement measurement(seq_number, current_id, actual_id, image_point);
                
                if (static_cast<size_t>(seq_number) < trajectory.size()) {
                    trajectory[seq_number].measurements.push_back(measurement);
                } else {
                    std::cerr << "Sequence number " << seq_number << " out of bounds for trajectory size " << trajectory.size() << " in " << meas_filepath << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Skipping point line in " << meas_filepath << " due to parsing error: " << line << " - " << e.what() << std::endl;
            }
        }
    }
}


std::vector<TrajPoint> load_trajectory(const std::string& folderpath_str) {
    fs::path folderpath(folderpath_str);
    if (!fs::exists(folderpath) || !fs::is_directory(folderpath)) {
        throw std::runtime_error("Folder not found or is not a directory: " + folderpath_str);
    }

    std::vector<TrajPoint> trajectory = _load_trajectory_without_meas((folderpath / "trajectory.dat").string());
    
    std::regex meas_file_regex("meas-\\d{5}\\.dat");

    for (const auto& entry : fs::directory_iterator(folderpath)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if (std::regex_match(filename, meas_file_regex)) {
                _add_measurements(trajectory, entry.path().string());
            }
        }
    }
    return trajectory;
}

Camera load_camera_data(const std::string& folderpath_str) {
    fs::path folderpath(folderpath_str);
    std::ifstream file((folderpath / "camera.dat").string());
    if (!file.is_open()) {
        throw std::runtime_error("File not found: " + (folderpath / "camera.dat").string());
    }

    Camera cam;
    std::string line;
    
    auto read_matrix3x3 = [&](Matrix3& matrix) {
        for (int i = 0; i < 3; ++i) {
            if (!std::getline(file, line)) throw std::runtime_error("Unexpected EOF reading matrix.");
            std::stringstream ss(line);
            for (int j = 0; j < 3; ++j) {
                if (!(ss >> matrix(i,j))) throw std::runtime_error("Error parsing matrix element.");
            }
        }
    };

    auto read_matrix4x4 = [&](Matrix4& matrix) {
        for (int i = 0; i < 4; ++i) {
            if (!std::getline(file, line)) throw std::runtime_error("Unexpected EOF reading matrix.");
            std::stringstream ss(line);
            for (int j = 0; j < 4; ++j) {
                if (!(ss >> matrix(i,j))) throw std::runtime_error("Error parsing matrix element.");
            }
        }
    };
    
    auto read_param = [&](const std::string& param_name, auto& value) {
        if (!std::getline(file, line) || line.find(param_name + ":") != 0) {
            throw std::runtime_error("Expected '" + param_name + ":'");
        }
        std::stringstream ss(line.substr(param_name.length() + 1));
        if (!(ss >> value)) {
            throw std::runtime_error("Error parsing value for " + param_name);
        }
    };

    try {
        // Camera Matrix
        if (!std::getline(file, line) || line != "camera matrix:") throw std::runtime_error("Missing 'camera matrix:' header");
        read_matrix3x3(cam.camera_matrix);

        // Camera Transform (Pose)
        if (!std::getline(file, line) || line != "cam_transform:") throw std::runtime_error("Missing 'cam_transform:' header");
        Matrix4 transform_matrix;
        read_matrix4x4(transform_matrix);
        cam.pose = Pose3(transform_matrix);
        
        // Scalar parameters
        read_param("z_near", cam.z_near);
        read_param("z_far", cam.z_far);
        read_param("width", cam.width);
        read_param("height", cam.height);

    } catch (const std::exception& e) {
        throw std::runtime_error("Error parsing camera.dat: " + std::string(e.what()));
    }

    return cam;
}

std::vector<Landmark> load_world_data(const std::string& folderpath_str) {
    fs::path folderpath(folderpath_str);
    std::ifstream file((folderpath / "world.dat").string());
    if (!file.is_open()) {
        throw std::runtime_error("File not found: " + (folderpath / "world.dat").string());
    }

    std::vector<Landmark> world;
    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        std::stringstream ss(line);
        std::vector<std::string> parts;
        std::string part;
        while (ss >> part) {
            parts.push_back(part);
        }

        if (parts.size() == 4) {
            try {
                int landmark_id = std::stoi(parts[0]);
                Scalar x = std::stod(parts[1]);
                Scalar y = std::stod(parts[2]);
                Scalar z = std::stod(parts[3]);
                world.emplace_back(Vector3(x,y,z), landmark_id);
            } catch (const std::exception& e) {
                std::cerr << "Skipping line in world.dat due to parsing error: " << line << " - " << e.what() << std::endl;
            }
        } else if (!line.empty()) {
             std::cerr << "Invalid line format in world.dat at line " << line_num << ": " << line << std::endl;
        }
    }
    return world;
}


Dataset load_dataset(const std::string& folderpath) {
    if (!fs::exists(folderpath) || !fs::is_directory(folderpath)) {
        throw std::runtime_error("Dataset folder not found or is not a directory: " + folderpath);
    }
    Dataset dataset;
    dataset.trajectory = load_trajectory(folderpath);
    dataset.camera = load_camera_data(folderpath);
    dataset.world = load_world_data(folderpath);
    return dataset;
}

// Example function similar to Python's main()
void run_dataset_loading_example(const std::string& data_dir_path) {
    std::cout << "Loading dataset from: " << data_dir_path << std::endl;
    try {
        Dataset dataset = load_dataset(data_dir_path);
        std::cout << "Dataset loaded successfully." << std::endl;
        std::cout << "Number of trajectory points: " << dataset.trajectory.size() << std::endl;
        std::cout << "Number of world landmarks: " << dataset.world.size() << std::endl;
        std::cout << "Camera width: " << dataset.camera.width << ", height: " << dataset.camera.height << std::endl;
        std::cout << "Camera Matrix:\n" << dataset.camera.camera_matrix << std::endl;

        if (!dataset.trajectory.empty()) {
            for (const auto& landmark : dataset.world) {
                std::cout << "Landmark ID: " << landmark.id << ", Position: (" 
                          << landmark.position.x() << ", " 
                          << landmark.position.y() << ", " 
                          << landmark.position.z() << ")" << std::endl;
            }

            for (const auto& traj_point : dataset.trajectory) {
                std::cout << "Trajectory Point ID: " << traj_point.id << ", Odometry: (" 
                          << traj_point.odometry.translation.x() << ", " 
                          << traj_point.odometry.translation.y() << "), Orientation: " 
                          << traj_point.odometry.rotation << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading dataset: " << e.what() << std::endl;
    }
}

} // namespace

#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_data_directory>" << std::endl;
        std::cerr << "Example: " << argv[0] << " /path/to/planar-monocular-slam/data" << std::endl;
        return 1;
    }
    std::string data_path = argv[1];
    
    pms::run_dataset_loading_example(data_path);
    
    return 0;
}