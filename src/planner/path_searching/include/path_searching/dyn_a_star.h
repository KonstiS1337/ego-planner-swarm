#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <queue>
#include <octomap/octomap.h>
#include <memory>

#define MAX_SHIFT_ITER 20

constexpr double inf = 1 >> 20;

struct Node
{
    octomap::point3d pos;
    double gScore;
    double fScore;
    std::shared_ptr<Node> parent;
    Node(const octomap::point3d &p, double g, double f, std::shared_ptr<Node> par = nullptr)
        : pos(p), gScore(g), fScore(f), parent(par) {}
};

class NodeComparator
{
public:
	bool operator()(const std::shared_ptr<Node>& node1,
                    const std::shared_ptr<Node>& node2) const
    {
        return node1->fScore > node2->fScore; // min-heap: smallest fScore first
    }
};

class AStar
{
private:
	std::shared_ptr<octomap::OcTree> octree_;
	bool isOccupied(const octomap::point3d& p) ;

	double getDiagHeu(const octomap::point3d& node1, const octomap::point3d& node2);
	double getEuclHeu(const octomap::point3d& node1, const octomap::point3d& node2);
	double getManhHeu(const octomap::point3d& node1, const octomap::point3d& node2);

	bool adjustStartEndPointsWithOctoMap(const Eigen::Vector3d& start_pt_in,const Eigen::Vector3d& end_pt_in,octomap::point3d& start_voxel,octomap::point3d& end_voxel);
	void retrievePath(std::shared_ptr<Node> goal_node,rclcpp::Time t_start,int iter);
	double getHeu(const octomap::point3d& node1, const octomap::point3d& node2)
	{
		return tie_breaker_ * getDiagHeu(node1, node2);
	}
	
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<octomap::point3d> gridPath_;

	std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>,NodeComparator> openSet_;

	int rounds_{0};
	int step_size_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initOctree(std::shared_ptr<octomap::OcTree> tree) {octree_ = tree;};

	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	//std::vector<Eigen::Vector3d> getPath(); //TOO do i need this for the outside?
};



#endif
