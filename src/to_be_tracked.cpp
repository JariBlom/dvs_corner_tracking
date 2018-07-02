#include "corner_event_detector/to_be_tracked.h"

namespace corner_event_detector
{

ToBeTracked::ToBeTracked(void)
{	
	// Distribute the n corners over an arc centered at location 
	tracked_corners = Eigen::MatrixXd::Zero(ncorners,2); // init matrix	
	const double angular_step = 2*pi/ncorners;
	const double angular_correction = angular_step/2;
	for (int i=0; i<ncorners; i++) 
	{
		// Define corners
		tracked_corners(i,0) = center_x + (int)cos(angular_step - angular_correction) * radius;
		tracked_corners(i,1) = center_y + (int)sin(angular_step - angular_correction) * radius;
		// Define offset from center per corner
		corner_offset[i] = radius;
	}
	// Define object center
	object_center[0] = center_x;
	object_center[1] = center_y; 

}

ToBeTracked::~ToBeTracked(void)
{
}

bool ToBeTracked::ClassifyEvent(int x, int y)
{
	// Loop through tracked corners to see if the event lies within the \
	search window of one of them
	bool in_window = false;
	for (int i=0; i<ncorners; i++) 
	{
		if (x > tracked_corners(i,0)-window_size && \
			x < tracked_corners(i,0)+window_size && \
			y > tracked_corners(i,1)-window_size && \
			y < tracked_corners(i,1)+window_size)
		{
			in_window = true;
			break;
		}
	}
	return in_window;
}

bool ToBeTracked::ClassifyCorner(int x, int y)
{
	// Calculate offset
	int offset = (int) sqrt ((object_center[0] - x)*(object_center[0] - x) \
		 + (object_center[1] - y)*(object_center[1] - y));
	// Loop through tracked corners to see if the corner is an outlier \
	with respect to the corners classified as belonging to the object
	bool in_window = true;	
	for (int i=0; i<ncorners; i++) 
	{
		if (offset > 2*corner_offset[i])
		{
			in_window = false;
			break;
		}
	}
	return in_window;
}

void ToBeTracked::Update(int x, int y)
{	
	// Find smallest distance to a corner
	int d_min = 2*window_size; // Initialize for loop
	int i_match = 0; // Matching corner
	for (int i=0; i<ncorners; i++)
	{
		// Calculate distance of corner to corners being tracked
		int distance = (int) sqrt ((x - tracked_corners(i,0))*(x - tracked_corners(i,0)) \
		 + (y - tracked_corners(i,1))*(y - tracked_corners(i,1)));
		// Check for the corner with shortest distance
		if (d_min > distance)
		{
			i_match = i;
			d_min = distance;
		}
	}
	// Update center
	object_center[0] = (int) (object_center[0]*ncorners-tracked_corners(i_match,0)+x)/ncorners;
	object_center[1] = (int) (object_center[1]*ncorners-tracked_corners(i_match,1)+x)/ncorners;	
	// Update offsets
	for (int i=0; i<ncorners; i++)
	{
		corner_offset[i] = (int) sqrt ((object_center[0] - tracked_corners(i,0))*(object_center[0] - tracked_corners(i,0)) \
		 + (object_center[1] - tracked_corners(i,1))*(object_center[1] - tracked_corners(i,1)));		
	}

	// Update the location of the matched corner
	int new_x = (x + tracked_corners(i_match,0))/2;
	int new_y = (y + tracked_corners(i_match,1))/2;
	tracked_corners(i_match,0) = new_x;
	tracked_corners(i_match,1) = new_y;

} 

}// namespace