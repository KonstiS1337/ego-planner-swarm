#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
}


double AStar::getDiagHeu(const octomap::point3d& node1, const octomap::point3d& node2)
    {
        // Calculate the differences in each coordinate (x, y, z)
        double dx = std::abs(node1.x() - node2.x());
        double dy = std::abs(node1.y() - node2.y());
        double dz = std::abs(node1.z() - node2.z());

        // Heuristic calculation based on diagonal distance
        double h = 0.0;
        int diag = std::min(std::min(dx, dy), dz);
        dx -= diag;
        dy -= diag;
        dz -= diag;

        // Handle the cases where dx, dy, dz are 0 for different combinations
        if (dx == 0)
        {
            h = 1.0 * std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(dy, dz) + 1.0 * std::abs(dy - dz);
        }
        if (dy == 0)
        {
            h = 1.0 * std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(dx, dz) + 1.0 * std::abs(dx - dz);
        }
        if (dz == 0)
        {
            h = 1.0 * std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(dx, dy) + 1.0 * std::abs(dx - dy);
        }

        return h;
    }


double AStar::getManhHeu(const octomap::point3d& node1, const octomap::point3d& node2)
    {
        // Calculate the differences in each coordinate (x, y, z)
        double dx = std::abs(node1.x() - node2.x());
        double dy = std::abs(node1.y() - node2.y());
        double dz = std::abs(node1.z() - node2.z());

        return dx + dy + dz;
    }

double AStar::getEuclHeu(const octomap::point3d& node1, const octomap::point3d& node2)
{
    return (node2 - node1).norm();
}

void AStar::retrievePath(std::shared_ptr<Node> goal_node,rclcpp::Time t_start, int iter)
{
    gridPath_.clear();
    for (auto node = goal_node; node != nullptr; node = node->parent)
    {
        gridPath_.push_back(node->pos);
    }
    std::reverse(gridPath_.begin(), gridPath_.end());

    double duration = (rclcpp::Clock().now() - t_start).seconds();
    RCLCPP_INFO(rclcpp::get_logger("AstarSearchOctomap"),
                "A* path found! %.3fs, %d iterations, %zu points.",
                duration, iter, gridPath_.size());
    return;
}

bool AStar::isOccupied(const octomap::point3d& p) {
    auto node = octree_->search(p);
    return node && octree_->isNodeOccupied(node);
}

bool AStar::adjustStartEndPointsWithOctoMap(
    const Eigen::Vector3d& start_pt_in,
    const Eigen::Vector3d& end_pt_in,
    octomap::point3d& start_voxel,
    octomap::point3d& end_voxel)
{
    // Convert Eigen::Vector3d → octomap::point3d
    Eigen::Vector3d start_pt = start_pt_in;
    Eigen::Vector3d end_pt   = end_pt_in;

    start_voxel = octomap::point3d(start_pt.x(), start_pt.y(), start_pt.z());
    end_voxel   = octomap::point3d(end_pt.x(), end_pt.y(), end_pt.z());


    // Helper to check map boundaries
    auto inBounds = [&](const octomap::point3d& p) {
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        return (p.x() >= min_x && p.x() <= max_x &&
                p.y() >= min_y && p.y() <= max_y &&
                p.z() >= min_z && p.z() <= max_z);
    };

    int iter = 0;

    // Move start point out of obstacle
    if (isOccupied(start_voxel))
    {
        Eigen::Vector3d dir = (start_pt - end_pt).normalized();
        while (isOccupied(start_voxel))
        {
            start_pt += dir * step_size_;
            start_voxel = octomap::point3d(start_pt.x(), start_pt.y(), start_pt.z());
            iter++;

            if (iter > MAX_SHIFT_ITER || !inBounds(start_voxel))
            {
                RCLCPP_WARN(rclcpp::get_logger("adjustStartEndPointsWithOctoMap"),
                            "No free voxel found for start point (max_iters reached or out of map)");
                return false;
            }
        }
    }

    iter = 0;
    // Move end point out of obstacle
    if (isOccupied(end_voxel))
    {
        Eigen::Vector3d dir = (end_pt - start_pt).normalized();
        while (isOccupied(end_voxel))
        {
            end_pt += dir * step_size_;
            end_voxel = octomap::point3d(end_pt.x(), end_pt.y(), end_pt.z());
            iter++;

            if (iter > MAX_SHIFT_ITER || !inBounds(end_voxel))
            {
                RCLCPP_WARN(rclcpp::get_logger("adjustStartEndPointsWithOctoMap"),
                            "No free voxel found for end point (max_iters reached or out of map)");
                return false;
            }
        }
    }

    return true;
}



bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{

    if (!octree_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AstarSearchOctomap"), "Octree is null!");
        return false;
    }
    rclcpp::Time t_start = rclcpp::Clock().now();
    ++rounds_; //TODO this seems not necessary

    step_size_ = step_size;

    octomap::point3d start_pt_shifted, end_pt_shifted;
    if (!adjustStartEndPointsWithOctoMap(start_pt, end_pt, start_pt_shifted, end_pt_shifted))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AstarSearch"), "Unable to handle the initial or end point, force return!");
        return false;
    }

    

    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeComparator> empty;
    openSet_.swap(empty);
    unordered_map<octomap::OcTreeKey, shared_ptr<Node>, octomap::OcTreeKey::KeyHash> visited;
    // Init start node for search algo


    auto startNode = make_shared<Node>(start_pt_shifted, 0.0, getHeu(start_pt_shifted, end_pt_shifted));
    openSet_.push(startNode);
    double resolution = octree_->getResolution();
    bool success = false;
    shared_ptr<Node> goalNode = nullptr;

    int iter = 0;

    while (!openSet_.empty())
    {
        iter++;
        auto current = openSet_.top();
        openSet_.pop();

        if ((current->pos - end_pt_shifted).norm() < resolution * 1.5)
        {
            success = true;
            goalNode = current;
            break;
        }
        // Generate 26 neighboring nodes (3D Moore neighborhood)
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue; //dont add the current voxel to the queue

                    octomap::point3d neighbor = current->pos + octomap::point3d(dx * resolution, dy * resolution, dz * resolution);
                    if (isOccupied(neighbor))
                        continue;

                    double cost = sqrt(dx * dx + dy * dy + dz * dz) * resolution;
                    double tentative_g = current->gScore + cost;

                    // Compute voxel key
                    octomap::OcTreeKey key;
                    if (!octree_->coordToKeyChecked(neighbor, key))
                        continue;

                    auto found = visited.find(key);
                    if (found == visited.end() || tentative_g < found->second->gScore) //if we not explored the node or if the gScore of an existing node is lower than the next computed cost we add the node to the list
                    {
                        auto neighborNode = make_shared<Node>(neighbor, tentative_g, tentative_g + getHeu(neighbor, end_pt_shifted), current);
                        visited[key] = neighborNode;
                        openSet_.push(neighborNode);
                    }
                }
        // Safety timeout (e.g. 0.2s max)
        if ((rclcpp::Clock().now() - t_start).seconds() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("AstarSearchOctomap"),
                        "A* search aborted (time limit exceeded). Iterations: %d", iter);
            return false;
        }
    }
    if (!success)
    {
        RCLCPP_WARN(rclcpp::get_logger("AstarSearchOctomap"), "A* failed to find a path after %d iterations.", iter);
        return false;
    }
    retrievePath(goalNode,t_start,iter);
    return true;
}        
