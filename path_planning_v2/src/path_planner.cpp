#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

#include <memory>

#include "utils.cpp"

namespace path_planner
{

    // Temp Vairables
    std::vector<Cone> newDetections = {
        Cone(point{1.1, 2.2}, 5, false, 0.95, "orange"),
        Cone(point{3.1, 4.2}, 3, false, 0.90, "blue"),
        Cone(point{5.1, 6.2}, 2, true, 0.85, "yellow")};

    std::vector<Cone> cones = {
        Cone(point{1.0, 2.0}, 5, false, 0.95, "orange"),
        Cone(point{3.0, 4.0}, 3, false, 0.90, "blue"),
        Cone(point{5.0, 6.0}, 2, true, 0.85, "yellow")};

    point car_pos = {0.0, 0.0};

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("path_planner");
        cones = compare_cones(newDetections, cones);
        std::vector<ConeBST> tlVector = track_limit_derivation(cones);
    }

    // Function to compare new con positions to historical cone positions
    std::vector<Cone> compare_cones(std::vector<Cone> new_cones, std::vector<Cone> confirmed_cones)
    {
        for (int i = 0; i < new_cones.size(); i++)
        {
            double closest_distance = 1000000;
            int closest_cone = 0;
            for (int j = 0; j < confirmed_cones.size(); j++)
            {
                double distance = distanceBetweenCones(new_cones[i], confirmed_cones[j]);
                if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_cone = j;
                }
            }

            if (closest_distance < margin_of_error)
            {
                // Update cone position
                confirmed_cones[closest_cone].weightedConePosUpdate(new_cones[i].getPos());
            }
            else
            {
                // Add new cone to list
                confirmed_cones.push_back(new_cones[i]);
            }
        }

        return confirmed_cones;

    }
    
    // a function to derive the left and right track limits from the confirmed cones
    std::vector<ConeBST> track_limit_derivation(std::vector<Cone> confirmed_cones)
    {
        ConeBST leftTrackLimit;
        ConeBST rightTrackLimit;

        leftTrackLimit.insert(closestLeftCone(confirmed_cones, car_pos));
        rightTrackLimit.insert(closestRightCone(confirmed_cones, car_pos));

        while (true)
        {
            // get the max element in the left track limit BST
            Cone leftMax = leftTrackLimit.findMax();

            // get the max element in the right track limit BST
            Cone rightMax = rightTrackLimit.findMax();


        }

        return {leftTrackLimit, rightTrackLimit};
    }
    //
    void loop_closure(Path open_loop)
    {

    }

    // a function that from the left and the right track limits, derive the midline of the track
    Path midline_derivation(ConeBST tlLeft, ConeBST tlRight)
    {
        
    }

    // a function that from the left and the right track limits and the midline of the track, derive the raceline
    Path raceline_derivation(ConeBST tlLeft, ConeBST tlRight, Path midline)
    {

    }
}