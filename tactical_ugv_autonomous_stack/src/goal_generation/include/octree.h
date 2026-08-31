// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 6th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This header file defines the Octree object used in the goal
// generation algorithm
// End File Decscription ################################################################################

#ifndef OCTREE_H
#define OCTREE_H

// List include files
#include <iostream> 
#include <fstream>
#include <vector> 
#include <Eigen/Dense>
#include <math.h>
#include <chrono>
#include <thread>
#include <algorithm>

#include <astar.h>

// List namespaces
using namespace std;

// Define number of directions (means of traversing from one voxel to another)
#define DIRECTIONS 24

// Local method
// linspace: Template which produces a vector of equally space points from "start" to "end" by "num" many points
// INPUTS: integer indicating the first element of the array,
// integer indicating the last element of the array,
// integer indicating the number of elements in the array
template <typename T> std::vector<T> linspace(int start, int end, int num)
{

	// Declare vector of template type
	std::vector<T> linspaced;

	// If the number of elements in the array is non-zero
	if (0 != num)
	{

		// If the number of elements is 1
		if (1 == num)
		{

			// Add the start point
			linspaced.push_back(static_cast<T>(start));

		} // if (1 == num)
		else
		{

			// Compute the stride needed
			double delta = (end - start) / (num - 1);

			// Iterate over the number of steps
			for (int i = 0; i < (num - 1); i++)
			{

				// Add the elements to the vector
				linspaced.push_back(start + delta * i);

			} // for (int i = 0; i < (num - 1); i++)

			// ensure that start and end are exactly the same as the input
			linspaced.push_back(static_cast<T>(end-1));

		} // if (1 == num)
	
	} // if (0 != num)

	return linspaced;

} // template <typename T> std::vector<T> linspace(int start, int end, int num)


inline void get_R321(Eigen::MatrixXf* att, Eigen::MatrixXf* R321)
{

	Eigen::MatrixXf R1 = Eigen::MatrixXf::Zero(3,3);
	R1 << 1.0, 0.0, 0.0,
		  0.0, cos((*att)(0)), -sin((*att)(0)), 
		  0.0, sin((*att)(0)), cos((*att)(0));
	Eigen::MatrixXf R2 = Eigen::MatrixXf::Zero(3,3);
	R2 << cos((*att)(1)), 0.0, sin((*att)(1)), 
	  0.0, 1.0, 0.0,
	  -sin((*att)(1)), 0.0, cos((*att)(1)); 
	Eigen::MatrixXf R3 = Eigen::MatrixXf::Zero(3,3);
	R3 << cos((*att)(2)), -sin((*att)(2)), 0.0, 
		  sin((*att)(2)), cos((*att)(2)), 0.0,
		  0.0, 0.0, 1.0;

  	(*R321) = R3*R2*R1;

}

// Define Octree class 
class Octree 
{

	// Integer arrays storing voxel cell transitions and their reversals
	int dx[DIRECTIONS] = {1, 0, -1, 0, 1,  1, -1, -1, 1, 0, -1,  0, 1,  1, -1, -1,  1,  0, -1,  0,  1,  1, -1, -1};
	int dy[DIRECTIONS] = {0, 0, 0,  0, 0,  0,  0,  0, 1, 1,  1,  1, 1,  1,  1,  1, -1, -1, -1, -1, -1, -1, -1, -1};
	int dz[DIRECTIONS] = {0, 1, 0, -1, 1, -1,  1, -1, 0, 1,  0, -1, 1, -1,  1, -1,  0,  1,  0, -1,  1, -1,  1, -1};
	int flip[DIRECTIONS] = {2, 3, 0, 1, 7, 6, 5, 4, 18, 19, 16, 17, 23, 22, 21, 20, 10, 11, 8, 9, 15, 14, 13, 12};

	// Declare Eigen::MatrixXf storing the explored points, the boundaries (two vertices) of the partitions, 
	// and newBounds which stores the new vertices of the eight children of a partition that is being divided
	Eigen::MatrixXf points, binBoundaries, newBounds = Eigen::MatrixXf::Zero(8, 6);
	
	// Integers representing the current number of partitions, \mu_3 the max number of explored voxels in a bin,
	// the max depth (the number of times the original partition can be divided), and
	// the maximum percentage of occupied voxels in a partition to the number of voxels
	// in that partition (\mu_4)
	int binCount, binCapacity, maxDepth, occupied_explored_threshold;

	// Float representing the minimum edge length of a partition, the maximum edge length of a partition, and
	// the maximum percentage of explored voxels to the total number of voxels in a parition
	float minSize, maxSize, percent_explored_threshold;

	// Vector of integers indicating a bin's depth (how many divisions of the original partition
	// are needed to get to this partition), and a vector storing the label of the parent to the partition
	// (the original partition is its own parent)
	std::vector<int> binDepths, binParents;

	// Declare string which stores the division style
	// There are currently two styles: normal (divides edges in half)
	// and "explored_weighted" (division occurs at the barycenter of explored voxels in the partition)
	string style;

	// Pointers to these matrices are instantiated using new, and currently 1000 blocks are
	// allocated. You may need more for certain parameter combinations and map sizes
	// Pointer to Eigen::MatrixXf each of which stores the coordinates points in the iTH pointers bin.
	Eigen::MatrixXf* points_in_bins = new Eigen::MatrixXf[5000];

	// Pointer to Eigen::VectorXi each of which store the labels of the points in the iTH pointers bin.
	Eigen::VectorXi* points_in_bins_labels = new Eigen::VectorXi[5000];
	
	// Vectors storing the volume of the iTH bin and the number of VOXELS in the iTH bin.
	std::vector<float> binVolume, numVoxels;
	
	// Pointer to Eigen::VectorXi each of which store the labels of the points in the iTH pointers bin.
	Eigen::VectorXi* binChildren = new Eigen::VectorXi[5000];

	// Defining a data type, Eigen::MatrixXb, i.e., dynamic matrix of booleans
	typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;

	// Defining a data type, Eigen::MatrixXb, i.e., dynamic matrix of unsigned short integers
	typedef Eigen::Matrix<unsigned short int, Eigen::Dynamic, Eigen::Dynamic> MatrixXusi;
	
	// Defining a data type, Eigen::MatrixXb, i.e., dynamic matrix of unsigned long integers
	typedef Eigen::Matrix<unsigned long int, Eigen::Dynamic, Eigen::Dynamic> MatrixXuli;

	// Length of side of a voxel (which is a cube) in meters.
	float voxel_resolution = 0.2;

	// Vector which encodes the new boundaries of child bins.
	std::vector<int> buildNewBoundaries = { 1,2,3,4,5,6,1,2,6,4,5,9,1,5,3,4,8,6,1,5,6,4,8,9,4,2,3,7,5,6,4,2,6,7,5,9,4,5,3,7,8,6,4,5,6,7,8,9 };

	// Vector containing the index of all bins which are leaves (not nodes)
	// and a vector containing all leaves which are goals
	std::vector<int> leaves, goalLeaves;

	// Vector of pairs (float and integer) storing the norm of the distance between the barycenter of the partition and the vehicle's position, and the label of the partition
	std::vector<pair<float, int>> binDist_label; 

	// Vector of pairs (float and integer) storing the volume of the partition, and the label of the partition
	std::vector<pair<float, int>> binVolume_children; 

	std::vector<pair<float, int>> binExploredRatio_vec; 
	std::vector<pair<int, int>> binUnexplored_vec; 

	std::vector<Eigen::MatrixXf> binExploredRatio_vec_centroids;

	// Vector of pairs (float and integer) storing the distance between the partition's centroid and the vehicle's position in meters, and the label of the partition. Will be sorted by the first element (by distance) 
	std::vector<pair<float, int>> similarVolume_sortByDistance;

	// Declare Eigen::MatrixXf storing the edge lengths, centroids, and distance between centroid and vehicle of goal leaves 
	Eigen::MatrixXf binEdgeLengths, binCentroids, binDistance;
	int numAttempts = 0;

	// User-defined tunables
	float mu_prox = 0.75, mu_max = 1 - mu_prox, mu_prox_temp = mu_prox, mu_max_temp = mu_max;

	Planner astar;
	bool success;

	float cone_radius = 7.0;
	float cone_height = 8.0;
	float cone_theta = atan(cone_radius/cone_height);

	Eigen::MatrixXf R321;

// Publically available members
public:

	// Constructor
	Octree()
	{

	} // Octree()

	// Constructor with arguments
	Octree(int x, int y, float p_e_t, int o_e_t, float minEdgeSize_input, float maxEdgeSize_input, float mu_prox_in, Eigen::MatrixXf *r, string style_, Eigen::MatrixXf *exM, Eigen::MatrixXf *quad_position, Eigen::MatrixXf *leafBoundaries, int* leaves_size, Eigen::MatrixXf* goalPosition, int (&map)[100][30][100], bool firstOctree, Eigen::MatrixXf *goal_history, bool* foundPreviousGoal, Eigen::MatrixXf *quad_attitude)
	{

		// Number of points is "x". 
		// "points" stores the 3D points to be binned by the OCTree: Rows: number of points - Columns: 4: x,y,z,occupancy status
		points = Eigen::MatrixXf::Zero(x, 4);

		// "binBoundaries" stores the boundaries of a bin in 3D space using two vertices. Rows: number of bins - Columns: 6 -- Columns 1:3 store first vertex, Columns 4:6 store the second vertex.
		// Note that here, there is only one bin. Later, this matrix is resized to reflect the current number of bins.
		// The first vertex is that closest to the origin (note that the points to be partitioned and the vertices of the partitions are defined in the first octant,
		// so all positive coordinates. This code must be modified to deal with the other octants)
		// The second vertex is furthest from the origin
		binBoundaries = Eigen::MatrixXf::Zero(1, 6);

		// Define the vertices of the original partition
		binBoundaries.block(0, 0, 1, 3) = (*r).col(0).transpose();
		binBoundaries.block(0, 3, 1, 3) = (*r).col(1).transpose();

		// Integer "binCount" keeps track of the number of bins in the OCTree
		binCount = 1;

		// "binParents" keeps track of which bin the current bin originated from.
		binParents.resize(binCount);

		// "binParents" is initialized to zero, since the first bin can be thought of as originating from itself.
		binParents[0] = 0;

		// "binCapacity" is user-defined, and is used as a condition for dividing a bin. If a bin stores more points than its capacity, it will be divided.
		binCapacity = y;

		//
		mu_prox = mu_prox_in;
		mu_max = 1-mu_prox;

		// Maximum percentage of explored voxels to total voxels in a partition
		percent_explored_threshold = p_e_t;

		// Maximum percentage of occupied voxels to total voxels in a partition
		occupied_explored_threshold = o_e_t;

		// "style" is user-defined, and is used to determine how the edges of the bins are determined.
		style = style_;

		// Properties for all OcTree nodes/leaves
		maxDepth = 1000000;	// Maximum number of times the original bin (the first bin thought of as originating from itself) can be divided.
		minSize = minEdgeSize_input;	// Minimum edge size in meters.
		maxSize = maxEdgeSize_input;	// Maximum edge size in meters.

		// Function inserts the points of interest into "Eigen::MatrixXf points".
		points.block(0,0,x,1) = (*exM).block(0,0,x,1);
		points.block(0,1,x,1) = (*exM).block(0,1,x,1);
		points.block(0,2,x,1) = (*exM).block(0,2,x,1);
		points.block(0,3,x,1) = (*exM).block(0,3,x,1);

		// Initialize the variable "points_in_bins_labels" with a vector of zeros, represented as integers
		points_in_bins_labels[0] = Eigen::VectorXi::Zero(points.rows());

		// Whatever order the points inserted into "Eigen::MatrixXf points" are currently in
		// give them a label corresponding to their row number.
		// This information is very important for tracking which points are in which bins.

		// Iterate over the number of points
		for (int i = 0; i < points.rows(); i++)
		{
			
			points_in_bins_labels[0][i] = i;

		} // for (int i = 0; i < points.rows(); i++)

		// Initialize the binVolume std::vector
		binVolume.resize(1);

		// Initialize the numVoxels std::vector
		numVoxels.resize(1);

		// Allocate memory, which is a function of the number of points.
		this->preallocateSpace();

		// The first bin is indexed by "1". The function "divide" takes a list of bin indices.
		// Declare vector of integers storing divisions
		std::vector<int> d;

		// Resize the vector 
		d.resize(1);

		// Set the first element to 1
		d[0] = 1;

		// The points in the original bin (which is the operating space) is every point in 
		// "points".
		points_in_bins[0] = points;

		// cout << "points.rows(): " << points.rows() << endl;

		// Start dividing the original bin
		this->divide(d);

		// std::cin.ignore();

		// *********************************************************************************************//
		// The octree algorithm has been used to partition the map according to some custom conditions 	//
		// Now we will use the partitions to determine a goal point 									//
		// *********************************************************************************************//

		// Write the boundaries of the bins to a text file, which can be read in Matlab using the matlab script
		// "plot_octree.m". (currently disabled)
		// TODO: Need to add binChildren to this file so we know which bin is at the lowest level.

		// cout << "binVolume.size(): " << binVolume.size()<< endl;

		//Iterate over the number of partitions
		for (int i = 0; i < binVolume.size(); i++)
		{
				// If a partition does not have 8 children, it is at the bottom of a branch, aka, a leaf
				if (binChildren[i].rows() < 7)
				{

					// Add the iTH partition to the list of leaves
					// This bin has no children, this is what we need.
					leaves.push_back(i);

					// Check if the partition has been substantially explored
					if ( ( points_in_bins[i].rows() ) / ( numVoxels[i] ) < p_e_t )
					{
						
						// This bin is relatively under-explored
						goalLeaves.push_back(i);

					} // if ( ( points_in_bins[i].rows() ) / ( numVoxels[i] ) < p_e_t )

				} // if (binChildren[i].rows() < 7)

		} // for (int i = 0; i < binVolume.size(); i++)

		// Record the number of leaves
		*leaves_size = leaves.size();

		// Integer representing the number of leaves that are goal candidates
		int goalLeaves_size = goalLeaves.size();

		// Set the leaves boundaries to a matrix of zeros
		*leafBoundaries = Eigen::MatrixXf::Zero(*leaves_size, 6);

		// Iterate over the number of leaves
		for (int i = 0; i < *leaves_size; i++)
		{
		
			// 	Set the leaf boundaries
			(*leafBoundaries).row(i) = binBoundaries.row(leaves[i]);
		
		} // for (int i = 0; i < *leaves_size; i++)

		// Set matrix to zeros, stores the edge lengths of candidate goal partitions 
		binEdgeLengths = Eigen::MatrixXf::Zero(goalLeaves_size, 3);
		
		// Set matrix to zeros, stores the centroids of candidate goal partitions 
		binCentroids = Eigen::MatrixXf::Zero(goalLeaves_size, 3);
		
		// Set matrix to zeros, stores the distance between the centroid of candidate goal partitions and vehicle's position 
		binDistance = Eigen::MatrixXf::Zero(goalLeaves_size, 3);
		
		// Iterate over the number of candidate partitions
		for (int i = 0; i < goalLeaves_size; i++)
		{		

			binExploredRatio_vec.push_back(make_pair(( points_in_bins[goalLeaves[i]].rows() ) / ( numVoxels[goalLeaves[i]] ), i));

			binUnexplored_vec.push_back(make_pair(numVoxels[goalLeaves[i]] - points_in_bins[goalLeaves[i]].rows(), i));


			// Compute the edge lengths of the candidate partitions
			binEdgeLengths.block(i, 0, 1, 3) = binBoundaries.block(goalLeaves[i], 3, 1, 3) - binBoundaries.block(goalLeaves[i], 0, 1, 3);

			// Compute the centroids of the candidate partitions
			binCentroids.block(i, 0, 1, 3) = binBoundaries.block(goalLeaves[i], 0, 1, 3) + (binEdgeLengths.block(i, 0, 1, 3) / 2);

			// cout << "binCentroids.block(i, 0, 1, 3): " << binCentroids.block(i, 0, 1, 3) << endl;
			// cout << "binUnex: " << binUnexplored_vec[i].first << endl;

			binExploredRatio_vec_centroids.push_back(binCentroids.block(i, 0, 1, 3));

			// Compute the volumes of the candidate partitions
			// binVolume_children.push_back(make_pair(binVolume[goalLeaves[i]], i));
			binVolume_children.push_back(make_pair(binUnexplored_vec[i].first, i));
			// binVolume_children.push_back(make_pair(binVolume[goalLeaves[i]], goalLeaves[i]));

			// Compute the distance between the centroid of the candidate partitions and the vehicle's position
			binDistance.block(i, 0, 1, 3) = (*quad_position).block(0, 0, 1, 3) - binCentroids.block(i, 0, 1, 3);

			// Add pair which stores the norm of the iTH binDistance and i, the index of the candidate partitions
			binDist_label.push_back(make_pair(binDistance.block(i, 0, 1, 3).norm(), i));
			// binDist_label.push_back(make_pair(binDistance.block(i, 0, 1, 3).norm(), goalLeaves[i]));

		} // for (int i = 0; i < goalLeaves_size; i++)

		// sort(binUnexplored_vec.rbegin(), binUnexplored_vec.rend());
		// sort(binUnexplored_vec.begin(), binUnexplored_vec.end());

		if (binVolume.size() == 1)
		{
			(*goalPosition) = (*goal_history).row((*goal_history).rows()-1);
			return;
		}

		// ******************************************************************************************** //
		// The block below resolves the ambiguity that arises due to partitions sharing similar volumes //
		// If two or more partitions share similar volume, they are sorted (when computing r_{\rm max})	//
		// by the distance between their centroids and the vehicle's position 							//  
		// ******************************************************************************************** //

		similarVolume_sortByDistance.resize(0);

		// If there are some leaf partitions
		if (binVolume_children.size() > 0)
		{
			
			// Sort the binVolume for the leaves by volume (the first element)
			sort(binVolume_children.rbegin(), binVolume_children.rend());

			// for (int i = 0; i < binVolume_children.size(); i++)
			// {
			// 	cout << "binVolume_children[" << i << "]: " << binVolume_children[i].first << ", " << binVolume_children[i].second << endl;
			// }

			// Iterate over the number of candidate partitions
			for (int i = 1; i < binVolume_children.size(); i++)
			{
				
				// If the volume is similar
				if (abs(binVolume_children[0].first - binVolume_children[i].first) < 3)
				{
					if (i == 1)
					{
						similarVolume_sortByDistance.push_back(make_pair(binDist_label[binVolume_children[0].second].first,binVolume_children[0].second));
					}
					
					// Add the norm of the distance between the centroid of the partition and the vehicle's position, and its index in the goalLeaves vector
					similarVolume_sortByDistance.push_back(make_pair(binDist_label[binVolume_children[i].second].first,binVolume_children[i].second));

				} // if (abs(binVolume_children[0].first - binVolume_children[i].first) < 0.032)
				else
				{

					// Since the binVolume_children vector is sorted, once we come across a partition with a dissimilar volume, we can stop checking
					break;

				} // if (abs(binVolume_children[0].first - binVolume_children[i].first) < 0.032)

			} // for (int i = 1; i < binVolume_children.size(); i++)

			// If there are multiple maximum volume partitions
			if (similarVolume_sortByDistance.size() > 0)
			{
				
				// sort the vector by the first element, the distance between the centroid of the partitions which are of max volume among the others and the vehicle's position
				sort(similarVolume_sortByDistance.begin(),similarVolume_sortByDistance.end());

				// sort the vector by the first element, the distance between the centroid of all candidate partitions and the vehicle's position
				sort(binDist_label.begin(), binDist_label.end());

				// Compute the goal position as the convex combination of the closest partition to the vehicle and the largest-closest partition
				*goalPosition = mu_prox * binCentroids.row(binDist_label[0].second) + mu_max * binCentroids.row(similarVolume_sortByDistance[0].second);

				// cout << "(*quad_position).block(0, 0, 1, 3): " << (*quad_position).block(0, 0, 1, 3) << endl;
				// cout << "binDist_label[0].first: " << binDist_label[0].first << endl;
				// cout << "binDist_label[0].second: " << binDist_label[0].second << endl;
				// cout << "binCentroids.row(binDist_label[0].second): " << binCentroids.row(binDist_label[0].second) << endl;
				// cout << "similarVolume_sortByDistance[0].first: " << similarVolume_sortByDistance[0].first << endl;
				// cout << "similarVolume_sortByDistance[0].second: " << similarVolume_sortByDistance[0].second << endl;
				// cout << " binCentroids.row(similarVolume_sortByDistance[0].second): " <<  binCentroids.row(similarVolume_sortByDistance[0].second) << endl;

			} // if (similarVolume_sortByDistance.size() > 0)
			else
			{

				// sort the vector by the first element, the distance between the centroid of all candidate partitions and the vehicle's position
				sort(binDist_label.begin(), binDist_label.end());

				// Compute the goal position as the convex combination of the closest partition to the vehicle and the largest partition
				*goalPosition = mu_prox * binCentroids.row(binDist_label[0].second) + mu_max * binCentroids.row(binVolume_children[0].second);

				// cout << "(*quad_position).block(0, 0, 1, 3): " << (*quad_position).block(0, 0, 1, 3) << endl;
				// cout << "binDist_label[0].first: " << binDist_label[0].first << endl;
				// cout << "binDist_label[0].second: " << binDist_label[0].second << endl;
				// cout << "binCentroids.row(binDist_label[0].second): " << binCentroids.row(binDist_label[0].second) << endl;
				// cout << "binVolume_children[0].first: " << binVolume_children[0].first << endl;
				// cout << "binVolume_children[0].second: " << binVolume_children[0].second << endl;
				// cout << " binCentroids.row(binVolume_children[0].second): " <<  binCentroids.row(binVolume_children[0].second) << endl;

			} // if (similarVolume_sortByDistance.size() > 0)

		
		} // if (binVolume_children.size() > 0)
		else
		{

			// Compute the goal position as the vehicle's position (there are no candidate partitions, mission is complete)
			*goalPosition = (*quad_position).block(0, 0, 1, 3);

			// Deallocate memory
			this->deallocateSpace();

			// Exit
			return;

		} // if (binVolume_children.size() > 0)

		// *************************************************** //
		// This block determines what partition the goal is in //
		// *************************************************** //

		Eigen::MatrixXf goaltMask = Eigen::MatrixXf::Zero(1,6);
		for (unsigned short int i = 0; i < goalLeaves_size; i++)
		{	

			// Iterate over the dimension of the operating space (3 dimensional)
				
			// Determine to which child bin the points in the parent bin belong.
			goaltMask(0, 0) = (*goalPosition)(0,0) >= binBoundaries(goalLeaves[i], 0) ? 1 : 0;
			goaltMask(0, 1) = (*goalPosition)(0,1) >= binBoundaries(goalLeaves[i], 1) ? 1 : 0;
			goaltMask(0, 2) = (*goalPosition)(0,2) >= binBoundaries(goalLeaves[i], 2) ? 1 : 0;
			goaltMask(0, 3) = (*goalPosition)(0,0) <= binBoundaries(goalLeaves[i], 3) ? 1 : 0;
			goaltMask(0, 4) = (*goalPosition)(0,1) <= binBoundaries(goalLeaves[i], 4) ? 1 : 0;
			goaltMask(0, 5) = (*goalPosition)(0,2) <= binBoundaries(goalLeaves[i], 5) ? 1 : 0;

			if (goaltMask.sum() == 6)
			{
				(*goalPosition) = binCentroids.block(i,0,1,3);
				break;
			}

		}

		// ************************************************************* // 
		// The block below checks if the goal coincides with an obstacle //
		// ************************************************************* // 

		int mx, my, mz;
		mx = (int)((*goalPosition)(0,0)/voxel_resolution);
		my = (int)((*goalPosition)(0,2)/voxel_resolution);
		mz = (int)((*goalPosition)(0,1)/voxel_resolution);

		Eigen::MatrixXf goalPosition_temp = Eigen::MatrixXf::Zero(1,3);

		// integer indicating the "radius" of the search for the new goal position (if necessary)
		int jj = 1;
		int ii = 1;

		cout << "(*goalPosition): " << (*goalPosition)<< endl;

		if (map[mz][my][mx] == 3 || map[mz][my][mx] == 2)
		{

			cout << "<OCTREE> Goal coincides with obstacle, computing new goal" << endl;

			// Boolean indicating that a new goal has not been found (or is not needed)
			bool foundGoodGoal = false;


			// While loop which exits if a goal has been found that does not coincide with an obstacle
			while( !foundGoodGoal )
			{

				jj = 1 + rand() % binDist_label.size();
				ii = 1 + rand() % binDist_label.size();

				// If there are multiple maximum volume partitions
				if (similarVolume_sortByDistance.size() > 0)
				{
					if (ii < similarVolume_sortByDistance.size())
					{
						// Compute the goal position as the convex combination of the closest partition to the vehicle and the largest-closest partition
						goalPosition_temp = mu_prox * binCentroids.row(binDist_label[0].second) + mu_max * binCentroids.row(similarVolume_sortByDistance[ii].second);
					}
					else
					{
						goalPosition_temp = mu_prox * binCentroids.row(binDist_label[jj].second) + mu_max * binCentroids.row(binVolume_children[0].second);
						// jj++;
					}

				} // if (similarVolume_sortByDistance.size() > 0)
				else
				{
					if (ii < binVolume_children.size())
					{
						// Compute the goal position as the convex combination of the closest partition to the vehicle and the largest partition
						goalPosition_temp = mu_prox * binCentroids.row(binDist_label[0].second) + mu_max * binCentroids.row(binVolume_children[ii].second);
					}
					else
					{
						goalPosition_temp = mu_prox * binCentroids.row(binDist_label[jj].second) + mu_max * binCentroids.row(binVolume_children[0].second);
						// jj++;
					}

				} // if (similarVolume_sortByDistance.size() > 0)

				mx = (int)(goalPosition_temp(0,0)/voxel_resolution);
				my = (int)(goalPosition_temp(0,2)/voxel_resolution);
				mz = (int)(goalPosition_temp(0,1)/voxel_resolution);

				if (map[mx][my][mz] != 3)
				{
					// Recompute the goal
					(*goalPosition)(0,0) = (float) mx*voxel_resolution;
					(*goalPosition)(0,2) = (float) my*voxel_resolution;
					(*goalPosition)(0,1) = (float) mz*voxel_resolution;

					// Indicate that a new goal has been found
					foundGoodGoal = true;
					cout << "Found better goal" << endl;

					// Break out of for loop
					break;

				} // if (map[mz][my][mx] != 3)

				// ii++;

			} // while( 1 )

		} // if (map[mx][my][mz] == 3)

		// ******************************************************* // 
		// This block determines if the goal has been found before //
		// ******************************************************* // 

		(*foundPreviousGoal) = false;

		if ((*goal_history).rows() > 0)
		{

			cout << "(*goalPosition): " << (*goalPosition) << endl;
			for (short int i = (*goal_history).rows()-1; i >= 0; i--)
			{
				if ( ( ( (*goalPosition) - (*goal_history).row(i) ).norm() <= 0.8 ) )
				{
					cout << "<OCTREE> found an old goal" << endl;
					(*foundPreviousGoal) = true;
					break;
				}
			}

		}

		ii = 1;
		jj = 1;

		numAttempts = 0;

		if ( (*foundPreviousGoal) )
		{

			bool fpg = false;

			if ((*goal_history).rows() > 0)
			{

				// for (int j = 0; j < binUnexplored_vec.size(); j++)
				// {
				// 	cout << " binUnexplored_vec: " << binVolume_children[j].first << "binVolume_children[j].sec: " << binVolume_children[j].second << endl;
				// }
				for (int j = 0; j < binUnexplored_vec.size(); j++)
				{
					fpg = false;
					*goalPosition = binCentroids.row(binVolume_children[j].second);
					// cout << "*goalPosition: " << *goalPosition << " binUnexplored_vec: " << binVolume_children[j].first << endl;
				
					for (int i = 0; i < (*goal_history).rows(); i++)
					{
						if ( ( ( (*goalPosition) - (*goal_history).row(i) ).norm() < 0.8 ) )
						{
							// cout << "<OCTREE> found something other than an old goal" << endl;
							fpg = true;
							break;
						}
					

					}

					if (!fpg)
					{
						(*foundPreviousGoal) = false;
						break;
					}

				}


			}


		}

		(*foundPreviousGoal) = false;

		// *********************************************** //
		// Use A* search to find a path to the goal point, //
		// confirming feasibility of the goal point        //
		// *********************************************** //

		if (firstOctree)
		{
			// Initialize the planner
			astar.initialize();
		}

		// Reset the success flag
		success = false;

		// Update the map, start, and goal
		success = astar.updatemap(map, quad_position, goalPosition);
		
		// If the update was successful
		if (success)
		{
			// Attempt to find a path
			success = astar.astar();

		} // if (success)
	
		int numFailures = 0;

		ii = 1;
		jj = 1;

		// If we were not successful in finding a path to the goal 
		if (!success)
		{

			designate_inaccessible_voxels(mx, my, mz, map);

			// While loop which exits when a feasible goal is found
			while(!success && numFailures < 3)
			{

				ii = 1 + rand() % binDist_label.size();
				jj = 1 + rand() % binDist_label.size();				

				// If there are multiple maximum volume partitions
				if (similarVolume_sortByDistance.size() > 0)
				{
					if (ii < similarVolume_sortByDistance.size())
					{
						// Compute the goal position as the convex combination of the closest partition to the vehicle and the largest-closest partition
						goalPosition_temp = mu_prox * binCentroids.row(binDist_label[0].second) + mu_max * binCentroids.row(similarVolume_sortByDistance[ii].second);
					}
					else
					{
						goalPosition_temp = mu_prox * binCentroids.row(binDist_label[jj].second) + mu_max * binCentroids.row(binVolume_children[0].second);
						// jj++;
					}

				} // if (similarVolume_sortByDistance.size() > 0)
				else
				{
					if (ii < binVolume_children.size())
					{
						// Compute the goal position as the convex combination of the closest partition to the vehicle and the largest partition
						goalPosition_temp = mu_prox * binCentroids.row(binDist_label[0].second) + mu_max * binCentroids.row(binVolume_children[ii].second);
					}
					else
					{
						goalPosition_temp = mu_prox * binCentroids.row(binDist_label[jj].second) + mu_max * binCentroids.row(binVolume_children[0].second);
						// jj++;
					}

				} // if (similarVolume_sortByDistance.size() > 0)

				mx = (int)(goalPosition_temp(0,0)/voxel_resolution);
				my = (int)(goalPosition_temp(0,2)/voxel_resolution);
				mz = (int)(goalPosition_temp(0,1)/voxel_resolution);				

				// Update the start and goal point
				astar.update_start_and_goal(quad_position, goalPosition);

				// Attempt to find a path
				success = astar.astar();

				if (!success)
				{

					// Designate the goal's voxel and adjacent voxels as inaccessible (for now, mark them occupied & explored)
					designate_inaccessible_voxels(mx, my, mz, map);

					ii++;
					numFailures++;
				}

				// ii++;

			} // while(!success)

			*goalPosition = goalPosition_temp;

		} // if (!success)

		cout << "Goal feasibility verified." << endl;

		// if (numFailures >= 3)
		// {
		// 	*goalPosition = binCentroids.row(binExploredRatio_vec.end()[-1].second);
		// }

		// ********************************************************** //
		// This block checks if the goal is dangerously above the UAV //
		// If so, the goal is projected onto the UAV's FOV 			  //
		// ********************************************************** //
		// Eigen::Vector2f goal_xy, quad_xy;
		// goal_xy(0) = (*goalPosition)(0,0);
		// goal_xy(1) = (*goalPosition)(0,1);

		// quad_xy(0) = (*quad_position)(0,0);
		// quad_xy(1) = (*quad_position)(0,1);

		// get_R321(quad_attitude, &R321);
		// Eigen::Vector3f x_axis = Eigen::Map<Eigen::Vector3f>(R321.col(0).data(),3);

		// if ( ((goal_xy-quad_xy).norm() <= 4.0) && (abs((*goalPosition)(0,2) - (*quad_position)(0,2)) >= 2.0) )
		// {

		// 	// Get orientation of focal axis
		// 	get_R321(quad_attitude, &R321);

		// 	// Define vector representing the UAV's focal axis
		// 	Eigen::Vector3f x_axis = Eigen::Map<Eigen::Vector3f>(R321.col(0).data(),3);
		// 	cout << "x_axis: " << x_axis << endl;
		// 	// cin.ignore();

		// 	// Define vector capturing difference in cone apex and goal position
		// 	Eigen::MatrixXf rg_temp = (*goalPosition) - (*quad_position);
		// 	Eigen::Vector3f rg =  Eigen::Map<Eigen::Vector3f>(rg_temp.data(),3);

		// 	cout << "rg: " << rg << endl;
		// 	// cin.ignore();

		// 	// Norm of rg
		// 	float rg_norm = rg.norm();

		// 	// Normalized rg
		// 	Eigen::Vector3f rgn = rg/rg_norm;

		// 	// Angle between rg and focal axis
		// 	float theta_rg_x = acos(rgn.dot(x_axis));

		// 	cout << "theta_rg_x: " << theta_rg_x << endl;
		// 	// cin.ignore();

		// 	// Length of adjacent side of triangle made by quad_position, goal_position, and projection of goal position
		// 	// onto focal axis
		// 	float c = cos(theta_rg_x)*rg_norm;

		// 	// Length of hypotnuse
		// 	float a = c/cos(cone_theta);

		// 	// Length of opposite
		// 	float b = a*sin(cone_theta);

		// 	// Projection of goal onto focal axis
		// 	Eigen::Vector3f cvec = Eigen::Map<Eigen::Vector3f>((*quad_position).data(),3) + c*x_axis;
		// 	Eigen::Vector3f cvec1 = c*x_axis;

		// 	cout << "cvec: " << cvec << endl;
		// 	cout << "cvec1: " << cvec1 << endl;
		// 	// cin.ignore();

		// 	// Difference between goal and projection of goal on focal axis
		// 	Eigen::Vector3f g_cvec = Eigen::Map<Eigen::Vector3f>((*goalPosition).data(),3) - cvec;

		// 	cout << "g_cvec: " << g_cvec << endl;
		// 	// cin.ignore();

		// 	// Projection of goal onto field of view cone
		// 	Eigen::Vector3f goaltemp;
		// 	if (rgn.dot(x_axis) > 0.1)
		// 	{			
		// 		goaltemp = cvec+b*g_cvec/(g_cvec.norm()); 
		// 		cout << "goaltemp: " << goaltemp << endl;
		// 		// cin.ignore();
		// 	}
		// 	else if (rgn.dot(x_axis) < -0.1)
		// 	{
		// 		goaltemp = cvec-2*cvec1-b*g_cvec/(g_cvec.norm()); 
		// 		cout << "goaltemp: " << goaltemp << endl;
		// 		// cin.ignore();
		// 	}
		// 	else
		// 	{
		// 		Eigen::MatrixXf cone_att = Eigen::MatrixXf::Zero(1,3);
		// 		cone_att(0) = 0.0;
		// 		cone_att(1) = cone_theta;
		// 		cone_att(2) = 0.0;
		// 		Eigen::MatrixXf newR321 = Eigen::MatrixXf::Zero(3,3);
		// 		get_R321(&cone_att, &newR321);
		// 		newR321 = newR321*R321;
		// 		x_axis = Eigen::Map<Eigen::Vector3f>(newR321.col(0).data(),3);
		// 		Eigen::Vector3f y_axis = Eigen::Map<Eigen::Vector3f>(R321.col(1).data(),3);

		// 		float theta1 = M_PI/2 - cone_theta;
		// 		float a1,a2,a3;
		// 		a1 = sin(theta1)*rg.norm();
		// 		a2 = tan(theta1)*((*goalPosition)(0,2)-(*quad_position)(0,2));
		// 		a3 = sqrt(a1*a1+a2*a2);

		// 		if (rgn.dot(y_axis) > 0.1)
		// 		{
		// 			Eigen::Vector3f x_perp;
		// 			x_perp(0) = -x_axis(1);
		// 			x_perp(1) = x_axis(0);
		// 			x_perp(2) = x_axis(2);
		// 			goaltemp = Eigen::Map<Eigen::Vector3f>((*goalPosition).data(),3) - a3*x_perp;
		// 		}	
		// 		else if (rgn.dot(y_axis) < -0.1)
		// 		{
		// 			Eigen::Vector3f y_axis = Eigen::Map<Eigen::Vector3f>(newR321.col(1).data(),3);
		// 			Eigen::Vector3f y_perp;
		// 			y_perp(0) = -y_axis(1);
		// 			y_perp(1) = y_axis(0);
		// 			y_perp(2) = y_axis(2);
		// 			goaltemp = Eigen::Map<Eigen::Vector3f>((*goalPosition).data(),3) - a3*y_perp;
		// 		}
		// 		else
		// 		{
		// 			cone_att(0) = cone_theta;
		// 			cone_att(1) = 0.0;
		// 			cone_att(2) = 0.0;
		// 			get_R321(&cone_att, &newR321);
		// 			newR321 = newR321*R321;
		// 			Eigen::Vector3f z_axis = Eigen::Map<Eigen::Vector3f>(newR321.col(2).data(),3);
		// 			a3 = sin(theta1)*rg.norm();
		// 			if ( (*goalPosition)(0,1) - (*quad_position)(0,1) > 0 )
		// 			{
		// 				goaltemp(0) = (*goalPosition)(0,0) + a3*z_axis(0);
		// 				goaltemp(1) = (*goalPosition)(0,1) - a3*z_axis(1);
		// 				goaltemp(2) = (*goalPosition)(0,2) + a3*z_axis(2);
		// 			}
		// 			else
		// 			{
		// 				goaltemp(0) = (*goalPosition)(0,0) + a3*z_axis(0);
		// 				goaltemp(1) = (*goalPosition)(0,1) + a3*z_axis(1);
		// 				goaltemp(2) = (*goalPosition)(0,2) + a3*z_axis(2);						
		// 			}
		// 		}

		// 		cout << "goaltemp: " << goaltemp << endl;
		// 		// cin.ignore();

		// 	}

		// 		(*goalPosition) = Eigen::Map<Eigen::MatrixXf>( goaltemp.data(), 1 , 3 );
		// 		(*goalPosition)(0,0) = floor(5*(*goalPosition)(0,0) + 0.5) / 5;
		// 		(*goalPosition)(0,1) = floor(5*(*goalPosition)(0,1) + 0.5) / 5;
		// 		(*goalPosition)(0,2) = floor(5*(*goalPosition)(0,2) + 0.5) / 5;
		// }

		// goaltMask = Eigen::MatrixXf::Zero(1,6);
		// for (unsigned short int i = 0; i < goalLeaves_size; i++)
		// {	

		// 	// Iterate over the dimension of the operating space (3 dimensional)
		// 	for (unsigned short int j = 0; j < 3; j++)
		// 	{
				
		// 		// Determine to which child bin the points in the parent bin belong.
		// 		goaltMask(0, j) = (*goalPosition)(0,j) >= binBoundaries(i, j) ? 1 : 0;
		// 		goaltMask(0, j+3) = (*goalPosition)(0,j) <= binBoundaries(i, j+3) ? 1 : 0;

		// 	} // for (unsigned short int j = 0; j < 3; j++)

		// 	if (goaltMask.sum() == 6)
		// 	{
		// 		(*goalPosition) = binCentroids.block(i,0,1,3);
		// 		break;
		// 	}

		// }	

		if (!(*foundPreviousGoal))
		{
			(*goalPosition)(0,0) = floor(5*(*goalPosition)(0,0) + 0.5) / 5;
			(*goalPosition)(0,1) = floor(5*(*goalPosition)(0,1) + 0.5) / 5;
			(*goalPosition)(0,2) = floor(5*(*goalPosition)(0,2) + 0.5) / 5;
			if ((*goalPosition)(0,0) > 20)
			{
				(*goalPosition)(0,0) = 19.8;
			} 
			else if ((*goalPosition)(0,0) < 0)
			{
				(*goalPosition)(0,0) = 0.2;
			}

			if ((*goalPosition)(0,1) > 20)
			{
				(*goalPosition)(0,1) = 19.8;
			} 
			else if ((*goalPosition)(0,1) < 0)
			{
				(*goalPosition)(0,1) = 0.2;
			}

			if ((*goalPosition)(0,2) >= 5.8)
			{
				(*goalPosition)(0,2) = 5.6;
			} 
			else if ((*goalPosition)(0,2) <= 0.2)
			{
				(*goalPosition)(0,2) = 0.4;
			}

			(*goal_history).conservativeResize((*goal_history).rows()+1,3);
			(*goal_history).row((*goal_history).rows()-1) = (*goalPosition);
		
		}
		cout << "goal_history: " << endl << (*goal_history) << endl;			



		// Deallocate space
		this->deallocateSpace();

	} // Octree(int x, int y, float p_e_t, int o_e_t, float minEdgeSize_input, float maxEdgeSize_input, float mu_prox_in, Eigen::MatrixXf *r, string style_, Eigen::MatrixXf *exM, Eigen::MatrixXf *quad_position, Eigen::MatrixXf *leafBoundaries, int* leaves_size, Eigen::MatrixXf* goalPosition, int (&map)[100][30][100])

	// Destructor
	~Octree()
	{

	} // ~Octree()


	// preallocateSpace: this function allocates memory based on an overestimate of the number of 
	// partitions that will be generated
	// INPUTS: none
	// OUTPUTS: none
	void preallocateSpace()
	{

		// Record the number of points
		int numPts = this->points.rows();

		// Record the number of bins initially as equal to the number of points
		int numBins = numPts;

		// If the binCapacity is finite
		if (isfinite((float)this->binCapacity))
		{
			
			// Compute the number of partitions assuming that each partition will contain half the binCapacity
			numBins = max( (int)ceil((2 * numPts) / this->binCapacity), 1);

		} // if (isfinite((float)this->binCapacity))

		// Preallocate some space to store the parent of a partition
		this->binParents.resize(numBins);
		
		// Iterate over the overestimation of the number of partitions
		for (unsigned short int i = 0; i < numBins; i++)
		{

			// initialize partition parent to zero
			this->binParents[i] = 0;

		} // for (unsigned short int i = 0; i < numBins; i++)

		// Resize the matrix storing the partition vertices
		this->binBoundaries.conservativeResize(numBins, 6);

		// Initialize all vertices to zero
		this->binBoundaries.block(1, 0, numBins - 1, 6) = Eigen::MatrixXf::Zero(numBins - 1, 6);

	} // void preallocateSpace()


	// deallocateSpace: this function deallocates memory that was allocated when the Octree was initially constructed
	// INPUTS: none
	// OUTPUTS: none
	void deallocateSpace()
	{

		// Resize matrices and vectors to 0x0
		this->binEdgeLengths.resize(0, 0);
		this->binBoundaries.resize(0, 0);
		this->points.resize(0, 0);
		this->binEdgeLengths.resize(0, 0);
		this->binCentroids.resize(0, 0);
		this->binDistance.resize(0, 0);
		this->binDist_label.resize(0);
		this->binVolume_children.resize(0);
		this->binExploredRatio_vec.resize(0);
		this->binExploredRatio_vec_centroids.resize(0);
		this->buildNewBoundaries.resize(0);
		this->leaves.resize(0);
		this->binChildren->resize(0);
		this->points_in_bins_labels->resize(0);
		this->points_in_bins->resize(0,0);

		// Delete pointers
		delete[] points_in_bins;
		delete[] binChildren;
		delete[] points_in_bins_labels;

		// Resize matrices and vectors to 0x0
		this->newBounds.resize(0,0);
		this->binParents.resize(0);
		this->numVoxels.resize(0);
		this->binVolume.resize(0);
		this->binDepths.resize(0);
	
	} // void deallocateSpace()


	// divide: checks conditions for dividing the current partition
	// INPUTS: vector of integers indicating the initial partitions
	// OUTPUTS: none
	void divide(std::vector<int> startingBins)
	{

		// unsigned short integer indicating the label of the partition being examined, and a variable to store the previous number of partitions
		unsigned short int binNo, oldCount;
		
		// integer storing the number of explored voxels in the partition, integer storing the number of occupied voxels in the partition
		int pointsinthisbin = 0, occupiedpointsinthisbin = 0;
		
		// Define an Eigen::MatrixXf storing the vertices of the current partition, an Eigen::MatrixXf storing the size of the edges of the current partition
		Eigen::MatrixXf thisBounds = Eigen::MatrixXf::Zero(1, 6), binEdgeSize = Eigen::MatrixXf::Zero(1, 3);
		
		//MatrixXb pointBins_eq_binNo = MatrixXb::Constant(this->pointBins.rows(), 1,false);
		
		// Floats storing the minimum and maximum edge lengths of the partition in meters 
		float minEdgeSize, maxEdgeSize;

		// Iterate over the number of starting partitions
		for (unsigned short int i = 0; i < startingBins.size(); i++)
		{

			// Set the binNo
			binNo = startingBins[i];
			// cout << "binNo: " << binNo-1 << endl;
			//if (this->binDepths[binNo - 1] + 1 >= this->maxDepth)
			//{
			//	continue;
			//}

			// Compute the vertices of the partition 
			thisBounds = this->binBoundaries.block(binNo - 1, 0, 1, 6);
			
			// Compute the length of the edges of the partition in meters
			binEdgeSize = thisBounds.block(0, 3, 1, 3) - thisBounds.block(0, 0, 1, 3);
			
			// Compute the minimum edge size
			minEdgeSize = binEdgeSize.minCoeff();

			// If the minimum edge size of the partition is smaller than the minimum allowable edge size, go to the next partition 
			if (minEdgeSize < minSize)
			{
				
				// Go to next iterator i
				continue;

			} // if (minEdgeSize < minSize)

			// Compute the maximum edge size 
			maxEdgeSize = binEdgeSize.maxCoeff();

			// Compute the old partition count
			oldCount = this->binCount;

			// Compute the number of explored voxels in this partition
			pointsinthisbin = points_in_bins[binNo - 1].rows();

			// Compute the number of occupied voxels in this partition
			occupiedpointsinthisbin = points_in_bins[binNo - 1].block(0,3,pointsinthisbin,1).sum();

			// Compute the volume of this partition (partitions are rectangular prisms, so their volume is the product of their edge lengths)
			binVolume[binNo - 1] = binEdgeSize.prod();

			// Compute the number of voxels in a partition (partition vertices are rounded to the nearest voxel_resolution, so they should contain an 
			// integer number of voxels, which can be deduced using the volume of a voxel and the volume of the partition)
			numVoxels[binNo - 1] = floor(binVolume[binNo - 1] / (voxel_resolution*voxel_resolution*voxel_resolution));

			// Could add more conditions here
			// If the percentage of occupied voxels is large, don't divide (or do divide)
			// If the percentage of unexplored voxels is large, don't divide, even if the number of explored voxels is > binCapacity

			// Save space by resizing variables that no longer have a use
			thisBounds.resize(0, 0);
			binEdgeSize.resize(0, 0);

			if (binNo - 1 == 0)
			{
				cout << "pointsinthisbin / numVoxels[0]: " << pointsinthisbin / numVoxels[binNo - 1] << endl;
				cout << "pet: " << percent_explored_threshold << endl;
				cout << "maxEdgeSize: " << maxEdgeSize << endl;
				cout << "this->maxSize: " << this->maxSize << endl;
				cout << "pointsinthisbin: " << pointsinthisbin << endl;
				cout << "this->binCapacity: " << this->binCapacity << endl;
			}

			// If the percentage of explored voxels to total voxels in the partition exceeds the maximum allowable, do not divide the partition
			if (pointsinthisbin / numVoxels[binNo - 1] >= percent_explored_threshold)
			{

				// Go to the next iterate
				continue;

			} // if (pointsinthisbin / numVoxels[binNo - 1] >= percent_explored_threshold)

			// If the largest edge length exceeds the maximum allowable, divide the partition 
			if (maxEdgeSize > this->maxSize || ( (pointsinthisbin > this->binCapacity) || (occupiedpointsinthisbin >= occupied_explored_threshold) ) )
			{

				// Define a vector storing the labels of the child partitions
				std::vector<int> newStartingBins = linspace<int>(oldCount + 1, oldCount + 9, 8);

				// New matrix storing the labels of the children of the current partition
				binChildren[binNo - 1] = Eigen::VectorXi::Zero(8);

				// Iterate over the number of children to produce (oct-)
				for (unsigned short int i = 0; i < 8; i++)
				{
				
					// Record the labels of the children of the current partition	
					binChildren[binNo - 1][i] = newStartingBins[i];
				
				} // for (unsigned short int i = 0; i < 8; i++)

				// Divide this partition
				divideBin(binNo, pointsinthisbin);

				// Recursive call to divide to check the child partitions, in case they need to be divided
				divide(newStartingBins);

				// Go to next iterate
				continue;

			} // if (maxEdgeSize > this->maxSize)

		} // for (unsigned short int i = 0; i < startingBins.size(); i++)

	} // void divide(std::vector<int> startingBins)


	// divideBin: this function computes the new vertices of the child partitions, 
	// determines which points in the parent belong to which child, updates necessary labels
	// INPUTS: integer indicating the partition to be divided,
	// integer indicating the number of points in the partition to be divided
	void divideBin(int binNo, int pointsinthisbin)
	{

		// Declare Eigen::MatrixXfs storing the old minimum edge length, maximum edge length, new vertices, and a matrix storing the previous 3
		Eigen::MatrixXf oldMin, oldMax, newDiv, minMidMax;
		
		// Vector of usi storing the label of the child that a point in the parent partition is now assigned
		std::vector<unsigned short int> binAssignment;
		
		// Define an Eigen::MatrixXf storing the sum of all coordinates of points in the parent partition (used in edge detector style for division)
		Eigen::MatrixXf numSum = Eigen::MatrixXf::Zero(3, 1);
		
		// Define an Eigen::MatrixXf storing the sum of all coordinates of points in the parent partition (used in edge detector style for division)
		Eigen::MatrixXf stdDev = Eigen::MatrixXf::Zero(3, 1);
		
		// Declare a vector of pairs of integers storing the binAssignment a to the ith point in the parent partition 
		std::vector<pair<int, int>> point_labels;
		
		// Define an Eigen::MatrixXusi storing a mask used in the determination of the bin assignment
		MatrixXusi gtMask = MatrixXusi::Constant(pointsinthisbin, 3, false);

		// Resize the binAssignment vector, every point must be assigned to a child partition
		// Note that the elements of binAssignment take on values [0,...,7]
		binAssignment.resize(pointsinthisbin);

		// Set oldMin and oldMax to 1x3 vector of zeros
		oldMin = Eigen::MatrixXf::Zero(1, 3);
		oldMax = Eigen::MatrixXf::Zero(1, 3);

		// Set oldMin and oldMax to the vertices of the parent partition
		oldMin = this->binBoundaries.block(binNo - 1, 0, 1, 3);
		oldMax = this->binBoundaries.block(binNo - 1, 3, 1, 3);

		// cout << "binNo: " << binNo-1 << endl;
		// cout << "pointsinthisbin: " << pointsinthisbin << endl;
		// cout << "This bin's origin: " << oldMin << " furthest vertex: " << oldMax << endl;

		// Set newDiv to 1x3 vector of zeros
		newDiv = Eigen::MatrixXf::Zero(1, 3);

		// Resize the binParents vector
		this->binParents.resize(this->binCount + 8);

		// Resize the binBoundaries matrix (so we can store the vertices of the children)
		this->binBoundaries.conservativeResize(this->binCount + 8, 6);

		// If the user selected the style "explored_average"
		if (this->style.compare("explored_average") == 0)
		{

			// Compute the newDivision based on the barycenter of the explored points in the partition
			newDiv = this->points_in_bins[binNo - 1].colwise().mean();

		} // if (this->style.compare("explored_average") == 0)
		else if (this->style.compare("normal") == 0)
		{
			// Otherwise if the user selected the style "normal"
		
			// Compute the newDivision by placing it at the centroid of the parent partition
			newDiv = (oldMin + oldMax) / 2;

		} // else if (this->style.compare("normal") == 0)


		/* This style will determine if explored points are grouped together using standard deviation of explored points in the bin
		If the standard deviation is low enough and number of explord points is high enough, perform corner detection using Harris detector
		Will have to blur the data in the box using a Gaussian filter or something (this way we do not detect the corners of the voxels)

		else if
		{
			for (int i = 0; i < pointsinthisbin; i++)
			{
				numSum += (points_in_bins[binNo - 1].row(i).transpose() - newDiv)*(points_in_bins[binNo - 1].row(i).transpose() - newDiv);
			}
			stdDev = ( numSum / (points_in_bins[binNo - 1].size() - 1)).sqrt();

			if (stdDev.norm() < sigma && pointsinthisbin/numVoxels[binNo - 1] > ligma)
			{
				do harris detector over the points in the bin to identify a corner
			}
		}

		*/

		// cout << "newDiv before rounding: " << newDiv << endl;

		// Round the new division to the nearest voxel_resolution (0.2m)
		newDiv(0,0) = floor(newDiv(0,0)*5 + 0.5)/5;
		newDiv(0,1) = floor(newDiv(0,1)*5 + 0.5)/5;
		newDiv(0,2) = floor(newDiv(0,2)*5 + 0.5)/5;
		// cout << "rounded newDiv: " << newDiv << endl;

		// Set the matrix to store the old minimum, the new division, and the old maximum vertex
		minMidMax = Eigen::MatrixXf::Zero(1, 9);
		minMidMax.block(0, 0, 1, 3) = oldMin;
		minMidMax.block(0, 3, 1, 3) = newDiv;
		minMidMax.block(0, 6, 1, 3) = oldMax;

		point_labels.clear();

		// Iterate over the number of children
		for (unsigned short int i = 1; i <= 8; i++)
		{

			// Iterate over the 6 coordinates needed to define a partition
			for (unsigned short int j = 1; j <= 6; j++)
			{

				// Compute vertices of the children
				newBounds(i - 1, j - 1) = minMidMax(0, this->buildNewBoundaries[j + (i - 1) * 6 - 1] - 1);

			} // for (unsigned short int j = 1; j <= 6; j++)

			// Update the matrix storing the new vertices
			this->binBoundaries.block(this->binCount + i - 1, 0, 1, 6) = newBounds.row(i - 1);

			// cout << "binBoundaries.block(this->binCount + i - 1, 0, 1, 6): " << this->binBoundaries.block(this->binCount + i - 1, 0, 1, 6) << endl;

			// Set the label of the parent for each of the child partitions
			this->binParents[this->binCount + i - 2] = binNo - 1;

			// cout << "binParents[this->binCount + i - 2]: " << this->binParents[this->binCount + i - 2] << endl; 
			// cout << "bin i:" << i << " max voxels: " << (this->binBoundaries.block(this->binCount + i - 1, 3, 1, 3) - this->binBoundaries.block(this->binCount + i - 1, 0, 1, 3)).prod()*125 << endl;

		} // for (unsigned short int i = 1; i <= 8; i++)


		int bins[8] = {0,0,0,0,0,0,0,0};

		// Iterate over all points in this partitions (binNo-1)
		for (unsigned long int i = 0; i < pointsinthisbin; i++)
		{

			// cout << "points_in_bins.row(" << i << "): " << points_in_bins[binNo - 1].row(i) << endl;

			// Iterate over the dimension of the operating space (3 dimensional)
			for (unsigned short int j = 0; j < 3; j++)
			{
				
				// Determine to which child bin the points in the parent bin belong.
				gtMask(i, j) = points_in_bins[binNo - 1](i, j) >= newDiv(0, j) ? 1 : 0;

			} // for (unsigned short int j = 0; j < 3; j++)

			// Get an integer which encodes the child bin containing the iTH point in the parent bin.
			binAssignment[i] = (2 * 2) * gtMask(i, 0) + (2 * 1) * gtMask(i, 1) + gtMask(i, 2);

			// for (unsigned short int j = 1; j <= 8; j++)
			// {
			// 	if ( points_in_bins[binNo - 1](i, 0) >= this->binBoundaries(this->binCount + j - 1,0) && points_in_bins[binNo - 1](i, 0) < this->binBoundaries(this->binCount + j - 1,3) && points_in_bins[binNo - 1](i, 1) >= this->binBoundaries(this->binCount + j - 1,1) && points_in_bins[binNo - 1](i, 1) < this->binBoundaries(this->binCount + j - 1,4) && points_in_bins[binNo - 1](i, 2) >= this->binBoundaries(this->binCount + j - 1,2) && points_in_bins[binNo - 1](i, 2) >= this->binBoundaries(this->binCount + j - 1, 5) )
			// 	{
			// 		binAssignment[i] = j-1;
			// 		break;		
			// 	}

			// } // for (unsigned short int j = 0; j < 3; j++)

			bins[binAssignment[i]]++;

			// cout << "oldMin, newDiv, oldMax" << endl;
			// cout << "minMidMax: " << minMidMax << endl; 
			// cout << "binAssignment: " << binAssignment[i] << endl;

			// Add new entry to the assigned bin and point label pair.
			point_labels.push_back(make_pair(binAssignment[i], points_in_bins_labels[binNo - 1](i)));

		} // for (unsigned long int i = 0; i < pointsinthisbin; i++)

		// int binSum = 0;
		// bool binMaxed = 0;
		// for (int i = 1; i <= 8; i++)
		// {
		// 	binMaxed = bins[i-1] > (this->binBoundaries.block(this->binCount + i - 1, 3, 1, 3) - this->binBoundaries.block(this->binCount + i - 1, 0, 1, 3)).prod()*125 ? 1 : 0;
		// 	cout << "Num bin assignments for bin i = "<<i-1<<": " << bins[i-1] << " :: " << binMaxed << endl;
		// 	binSum+=bins[i-1];
		// }
		// cout << "sanity check: total bin assignments: " << binSum << endl;


		// cin.ignore();

		// Sort the pair "point_labels" according to the first element "binAssignment", keeping
		// the point label handy.
		sort(point_labels.begin(), point_labels.end());

		// If for some reason, the point labels is larger than the number of voxels in the map, resize it
		if (point_labels.size() > 300000)
		{
			
			// Resize the vector
			point_labels.resize(300000);

		} // if (point_labels.size() > 300000)

		// Integers to keep track of number of points allocated to a particular partition 
		int count = 0, countt = 0;

		// Declare Eigen::MatrixXf to store temporarily a point to be added to the list of points in a child partition
		Eigen::MatrixXf temp;

		// Declare Eigen::VectorXi to store the point labels
		Eigen::VectorXi temp1;

		// Iterate over the number of child partitions
		for (unsigned short int i = 0; i < 8; i++)
		{

			// The number of points in a child partition is at most the number of points in the parent partition,
			// set the temporary matrix and vector to zeros 
			temp = Eigen::MatrixXf::Zero(pointsinthisbin, 4);
			temp1 = Eigen::VectorXi::Zero(pointsinthisbin);

			// While loop which breaks when the value of the ith point_labels changes while iterating through the sorted list of point_labels 
			// point_labels -- binAssignment -- points_in_bins_labels
			// 1 -- 0, 1000
			// 2 -- 0, 1001
			// 3 -- 0, 9026
			// 4 -- 0, ...
			// 5 -- 0
			// 6 -- 0
			// ...
			// 9 -- 0 
			// 10 -- 1 <- here is where the while loop breaks, we have finished adding the points which have been assigned to child partition 0, we can move on to 1
			// 11 -- 1
			// 12 -- 1
			// ...
			// 26 -- 2
			// 27 -- 2
			// ...
			while (1)
			{

				// If the binAssignment of the (count-1)TH is equal to i and count is a feasible value
				if ((point_labels[count].first == i) && (count < point_labels.size()))
				{
					
					// Increase countt (keeps track of the number of explored points added to a child partition)
					countt++;

					// Add the explore points coordinates and occupancy status to the temporary variable
					temp.row(countt - 1) = this->points.row(point_labels[count].second);

					// Add the label of the explored point to the temporary variable
					temp1(countt - 1) = point_labels[count].second;

					// Increase count (keeps track of which explore point to examing, since an explored point can only be assigned to one child partition this is not reset until divideBin exits)
					count++;

				} // if ((point_labels[count - 1].first == i) && (count < point_labels.size()))
				else
				{
					
					// cout << "Points added to " << i+ this->binCount << ": " << countt << endl;
					// Otherwise, we have reached the end of the block of points that have been assigned to the iTH child partition,
					// post the explore points and their labels
					// cout << "points_in_bins[" << i+this->binCount << "] size before adding: " << points_in_bins[i + this->binCount].rows() << endl;

					// If count is feasible
					if (count < 300000)
					{

						// The number of points in a partition is likely less than that of the parent, so conservatively resize the partition (resize while retaining values)
						temp.conservativeResize(countt, 4);
						temp1.conservativeResize(countt);

						// binMaxed = ((float) bins[i]) > (this->binBoundaries.block(this->binCount + i, 3, 1, 3) - this->binBoundaries.block(this->binCount + i, 0, 1, 3)).prod()*125 ? 1 : 0;
						// // cout << "Num bin assignments for bin i = "<<i-1<<": " << bins[i-1] << " :: " << binMaxed << endl;
						// if (binMaxed)
						// {
						// 	cout << "Parent bin's origin: " << oldMin << " furthest vertex: " << oldMax << endl;
						// 	cout << "newDiv: " << newDiv << endl;
						// 	cout << "points: " << endl << temp << endl;
						// }

						// Post the explored points coordinates and occupancy status
						this->points_in_bins[i + this->binCount] = temp;

						// Resize the matrix storing that information
						this->points_in_bins[i + this->binCount].conservativeResize(countt, 4);

						// Post the labels of the explored points
						this->points_in_bins_labels[i + this->binCount] = temp1;

						// Resize the matrix storing that information
						this->points_in_bins_labels[i + this->binCount].conservativeResize(countt, 1);

						// cout << "points_in_bins #: " << i+this->binCount << " size: " << temp.rows() << " label size: " << temp1.rows() << endl;

						// Reset countt
						countt = 0;

					} // if (count < 300000)
					else
					{
						
						// The number of points in a partition is likely less than that of the parent, so conservatively resize the partition (resize while retaining values)
						temp.conservativeResize(300000, 4);

						// Post the explored points coordinates and occupancy status
						this->points_in_bins[i + this->binCount] = temp;

						// Resize the matrix storing that information
						this->points_in_bins[i + this->binCount].conservativeResize(300000, 4);

						// Post the labels of the explored points
						this->points_in_bins_labels[i + this->binCount] = temp1;

						// Resize the matrix storing that information
						this->points_in_bins_labels[i + this->binCount].conservativeResize(300000, 1);
						
						// Reset countt
						countt = 0;			

					} // if (count < 300000)
					
					break;

				} // if ((point_labels[count - 1].first == i) && (count < point_labels.size()))

			} // while (1)

		} // for (unsigned short int i = 0; i < 8; i++)

		// cin.ignore();

		// Increase the number of partitions
		this->binCount = this->binCount + 8;

		// Deallocate space
		binVolume.resize(binVolume.size() + 8);
		numVoxels.resize(binVolume.size() + 8);
		temp.resize(0,0);
		temp1.resize(0);
		newDiv.resize(0,0);
		oldMax.resize(0,0);
		oldMin.resize(0,0);
		minMidMax.resize(0,0);
		point_labels.resize(0);
		binAssignment.resize(0);

	} // void divideBin(int binNo, int pointsinthisbin)

	void designate_inaccessible_voxels(int z, int y, int x, int (&map)[100][30][100])
	{
		for (int d = 0; d < DIRECTIONS; d++)
		{
			if (x+dx[d] >= 0 && x+dx[d] < 100 && y+dy[d] >= 0 && y+dy[d] < 30 && z+dz[d] >= 0 && z+dz[d] < 100)
			{
				map[x+dx[d]][y+dy[d]][z+dz[d]] = 4; // 4 can be interpreted as explored and occupied, or inaccessible
			}
		}

		cout << "Marked points inaccessible" << endl;
	}

}; // class Octree 

#endif