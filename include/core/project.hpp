#include <pcl/filters/project_inliers.h>

void projectPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    output->clear();
    if (input->empty())
        return;

    // // 遍历所有点，将Z坐标强制设为投影平面的Z值（X/Y保持不变）
    // for (const auto &point : *input)
    // {
    //     pcl::PointXYZ projected_point;
    //     projected_point.x = point.x;             // X坐标不变
    //     projected_point.y = point.y;             // Y坐标不变
    //     projected_point.z = projection_plane_z_; // Z坐标投影到指定平面
    //     output->push_back(projected_point);
    // }

    // /***********template 6**************/

    // /***********************************/
    pcl::ModelCoefficients::Ptr ground(new pcl::ModelCoefficients);
    ground->values = {0.0, 0.0, 1.0, -0.7}; // [a,b,c,d]

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(input);
    proj.setModelCoefficients(ground);
    proj.setCopyAllData(false); // 仅输出投影后的内点
    proj.filter(*output);
}