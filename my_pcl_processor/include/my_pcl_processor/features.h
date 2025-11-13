#ifndef FEATURES_H
#define FEATURES_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

// Define point type
typedef pcl::PointXYZRGB PointT;

/**
 * @brief Computes a 3D color histogram
 * @param cloud The cluster to analyze
 * @param nbins The number of bins for each channel (R, G, B)
 * @return A normalized 1D vector of histogram features
 */
std::vector<float> computeColorHistogram(const pcl::PointCloud<PointT>::Ptr& cloud, int nbins = 16)
{
    // Compute the min/max of x,y,z
    float r_min = 0.0, r_max = 255.0;
    float g_min = 0.0, g_max = 255.0;
    float b_min = 0.0, b_max = 255.0;

    // 1. Calculate color bins
    std::vector<float> hist(nbins * nbins * nbins, 0.0);
    float r_bin_size = (r_max - r_min) / nbins;
    float g_bin_size = (g_max - g_min) / nbins;
    float b_bin_size = (b_max - b_min) / nbins;

    for (const auto& point : cloud->points)
    {
        int r_bin = std::min(nbins - 1, static_cast<int>(point.r / r_bin_size));
        int g_bin = std::min(nbins - 1, static_cast<int>(point.g / g_bin_size));
        int b_bin = std::min(nbins - 1, static_cast<int>(point.b / b_bin_size));
        hist[r_bin * nbins * nbins + g_bin * nbins + b_bin]++;
    }

    // 2. Normalize histogram
    float sum = 0;
    for (float val : hist) sum += val;
    if (sum > 0)
    {
        for (float& val : hist) val /= sum;
    }

    return hist;
}

/**
 * @brief Computes surface normals for a cloud
 * @param cloud The cluster to analyze
 * @return A cloud of normals
 */
pcl::PointCloud<pcl::Normal>::Ptr getNormals(const pcl::PointCloud<PointT>::Ptr& cloud)
{
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // Use a radius search
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    return normals;
}

/**
 * @brief Computes a 3D histogram of normals
 * @param normals The normals cloud to analyze
 * @param nbins The number of bins for each channel (x, y, z)
 * @return A normalized 1D vector of histogram features
 */
std::vector<float> computeNormalHistogram(const pcl::PointCloud<pcl::Normal>::Ptr& normals, int nbins = 8)
{
    // 1. Calculate normal bins
    std::vector<float> hist(nbins * nbins * nbins, 0.0);
    float x_min = -1.0, x_max = 1.0;
    float y_min = -1.0, y_max = 1.0;
    float z_min = -1.0, z_max = 1.0;

    float x_bin_size = (x_max - x_min) / nbins;
    float y_bin_size = (y_max - y_min) / nbins;
    float z_bin_size = (z_max - z_min) / nbins;

    for (const auto& normal : normals->points)
    {
        int x_bin = std::min(nbins - 1, static_cast<int>((normal.normal_x - x_min) / x_bin_size));
        int y_bin = std::min(nbins - 1, static_cast<int>((normal.normal_y - y_min) / y_bin_size));
        int z_bin = std::min(nbins - 1, static_cast<int>((normal.normal_z - z_min) / z_bin_size));
        hist[x_bin * nbins * nbins + y_bin * nbins + z_bin]++;
    }

    // 2. Normalize histogram
    float sum = 0;
    for (float val : hist) sum += val;
    if (sum > 0)
    {
        for (float& val : hist) val /= sum;
    }

    return hist;
}

#endif // FEATURES_H