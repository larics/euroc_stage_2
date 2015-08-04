import roslib

try:
    roslib.load_manifest('rospkg');
except:
    pass
import rospkg

def findSubmodulePaths(verbose=False):
    # Try to find all aslam submodules
    submoduleNames = ["aslam_cv",
                      "aslam_fiducials",
                      "aslam_fixed_lag_estimation",
                      "aslam_incremental_calibration",
                      "aslam_install",
                      "aslam_nonparametric_estimation",
                      "aslam_offline_calibration",
                      "aslam_optimizer",
                      "aslam_posegraph",
                      "aslam_python",
                      "aslam_sample_consensus",
                      "aslam_simulation",
                      "aslam_vcharge",
                      "aslam_visual_inertial",
                      "aslam_vo",
                      "ceres",
                      "ethzasl_brisk",
                      "g2o",
                      "geometric_vision",
                      "matrixvision_camera",
                      "numpy_eigen",
                      "optimizer_speed_test",
                      "schweizer_messer"]
    #packages, not_found = rospack.expand_to_packages( submoduleNames, rospack.RosPack(), rospack.RosStack())
    
    paths = rospkg.get_ros_paths()
    
    submodules = {}
    for nm in submoduleNames:
        indices = [i for i, s in enumerate(paths) if nm in s]
        if len(indices) == 1:
            submodules[nm] = paths[indices[0]]
        elif len(indices) > 1:
            print "Error: More than one potential path was found for the repository {0}".format(nm)
            for i in indices:
                print " - {0}".format(paths[i])
        elif verbose:
            print "Unable to find the submodule {0}".format(nm)
    return submodules

        
    
