#!/usr/bin/env python3
import rospy

from optimize_path.srv import PathFitInfo
from optimize_path.srv import PathFitInfoResponse

from func_optimize import path_generation

def handelclient(rq):
    print("**** receive Request !! ****\n")
    return PathFitInfoResponse(
        path_generation(rq.fit_degree, rq.path_seg_num, rq.prime_path,
                rq.path_resolution))


def server_func():
    rospy.init_node("optimize_server")
    s = rospy.Service("hybrid_A/compute_path/optimize_service", PathFitInfo, handelclient)
    print("waiting for req ...")
    s.spin()


if __name__ == "__main__":
    server_func()