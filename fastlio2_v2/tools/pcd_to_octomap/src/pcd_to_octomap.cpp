#include <octomap/octomap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <cmath>

int main(int argc, char** argv){
    if(argc < 4){
        std::cerr << "Usage: pcd_to_octomap <input.pcd> <output.bt> <resolution>\n";
        return 1;
    }
    const std::string infile = argv[1];
    const std::string outfile = argv[2];
    double resolution = atof(argv[3]);
    if(resolution <= 0){
        std::cerr << "Resolution must be > 0" << std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(infile, cloud) == -1){
        std::cerr << "Couldn't read file " << infile << std::endl;
        return 1;
    }

    octomap::OcTree tree(resolution);
    for(const auto &p : cloud.points){
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }
    tree.updateInnerOccupancy();

    if(!tree.writeBinary(outfile)){
        std::cerr << "Failed to write OctoMap to " << outfile << std::endl;
        return 1;
    }

    std::cout << "Wrote OctoMap binary to: " << outfile << std::endl;
    return 0;
}
