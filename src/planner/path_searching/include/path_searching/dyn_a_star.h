#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <queue>
#include <octomap/octomap.h>
#include <memory>
#include <octomap_msgs/msg/octomap.hpp>

#define MAX_SHIFT_ITER 20

constexpr double inf = 1 >> 20;

struct Node
{
    octomap::point3d pos;
    double gScore;
    double fScore;
    std::shared_ptr<Node> parent;
	bool expanded
    Node(const octomap::point3d &p, double g, double f, shared_ptr<Node> par = nullptr)
        : pos(p), gScore(g), fScore(f), parent(par) {}
};

class NodeComparator
{
public:
	bool operator()(const Node *node1,const Node *node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;
	octomap::OcTree octree_;
	bool isOccupied(const octomap::point3d& p) ;

	double getDiagHeu(const octomap::point3d& node1, const octomap::point3d& node2);
	double getEuclHeu(const octomap::point3d& node1, const octomap::point3d& node2);
	double getManhHeu(const octomap::point3d& node1, const octomap::point3d& node2)
	inline double getHeu(const octomap::pointed3d& node1, const octomap::point3d& node2);

	vooid retrievePath(GridNodePtr current);

	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<*octomap::point3d> gridPath_;

	std::priority_queue<Node, std::vector<Node>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	std::vector<Eigen::Vector3d> getPath(); //TOO do i need this for the outside?
};

inline double AStar::getHeu(const octomap::point3d& node1, const octomap::point3d& node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}


#endif
