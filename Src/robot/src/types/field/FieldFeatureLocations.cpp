#include "types/field/FieldFeatureLocations.hpp"

#include "utils/defs/FieldDefinitions.hpp"
#include "utils/math/basic_maths.hpp"

#include <vector>

FieldFeatureLocations::FieldFeatureLocations() {
    // !! NOTE !!
    // The following comments describe field features as if we're looking at Offnao, but
    // the orientation is defined in the field coordinate conventions.
    corners.push_back(FieldFeature(-FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, -M_PI_4));                                  // Left Top
    corners.push_back(FieldFeature(-FIELD_LENGTH / 2.0 + GOAL_BOX_LENGTH, GOAL_BOX_WIDTH / 2.0, -3.0 * M_PI_4));       // Left Goal Top
    corners.push_back(FieldFeature(-FIELD_LENGTH / 2.0 + PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH / 2.0, -3.0 * M_PI_4)); // Left Penalty Top
    corners.push_back(FieldFeature(-FIELD_LENGTH / 2.0 + GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH / 2.0, 3.0 * M_PI_4));       // Left Goal Bottom
    corners.push_back(FieldFeature(-FIELD_LENGTH / 2.0 + PENALTY_BOX_LENGTH, -PENALTY_BOX_WIDTH / 2.0, 3.0 * M_PI_4)); // Left Penalty Bottom
    corners.push_back(FieldFeature(-FIELD_LENGTH / 2.0, -FIELD_WIDTH / 2.0, M_PI_4));                                  // Left Bottom
    corners.push_back(FieldFeature(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, -3.0 * M_PI_4));                             // Right Top
    corners.push_back(FieldFeature(FIELD_LENGTH / 2.0 - GOAL_BOX_LENGTH, GOAL_BOX_WIDTH / 2.0, -M_PI_4));              // Right Goal Top
    corners.push_back(FieldFeature(FIELD_LENGTH / 2.0 - PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH / 2.0, -M_PI_4));        // Right Penalty Top
    corners.push_back(FieldFeature(FIELD_LENGTH / 2.0 - GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH / 2.0, M_PI_4));              // Right Goal Bottom
    corners.push_back(FieldFeature(FIELD_LENGTH / 2.0 - PENALTY_BOX_LENGTH, -PENALTY_BOX_WIDTH / 2.0, M_PI_4));        // Right Penalty Bottom
    corners.push_back(FieldFeature(FIELD_LENGTH / 2.0, -FIELD_WIDTH / 2.0, 3.0 * M_PI_4));                             // Right Bottom

    t_junctions.push_back(FieldFeature(-FIELD_LENGTH / 2.0, GOAL_BOX_WIDTH / 2.0, 0));       // Left Goal top
    t_junctions.push_back(FieldFeature(-FIELD_LENGTH / 2.0, PENALTY_BOX_WIDTH / 2.0, 0));    // Left Penalty top
    t_junctions.push_back(FieldFeature(-FIELD_LENGTH / 2.0, -GOAL_BOX_WIDTH / 2.0, 0));      // Left Goal bottom
    t_junctions.push_back(FieldFeature(-FIELD_LENGTH / 2.0, -PENALTY_BOX_WIDTH / 2.0, 0));   // Left Penalty bottom
    t_junctions.push_back(FieldFeature(0, FIELD_WIDTH / 2.0, -M_PI_2));                      // Centre Top
    t_junctions.push_back(FieldFeature(0, -FIELD_WIDTH / 2.0, M_PI_2));                      // Centre Bottom
    t_junctions.push_back(FieldFeature(FIELD_LENGTH / 2.0, GOAL_BOX_WIDTH / 2.0, M_PI));     // Right Goal Top
    t_junctions.push_back(FieldFeature(FIELD_LENGTH / 2.0, PENALTY_BOX_WIDTH / 2.0, M_PI));  // Right Penalty Top
    t_junctions.push_back(FieldFeature(FIELD_LENGTH / 2.0, -GOAL_BOX_WIDTH / 2.0, M_PI));    // Right Goal Bottom
    t_junctions.push_back(FieldFeature(FIELD_LENGTH / 2.0, -PENALTY_BOX_WIDTH / 2.0, M_PI)); // Right Penalty Bottom

    x_junctions.push_back(FieldFeature(0, CENTER_CIRCLE_DIAMETER / 2.0, 0));  // Top X Junction
    x_junctions.push_back(FieldFeature(0, -CENTER_CIRCLE_DIAMETER / 2.0, 0)); // Bottom X Junction

    centre_circles.push_back(FieldFeature(0, 0, M_PI_2));  // Orientation 1
    centre_circles.push_back(FieldFeature(0, 0, -M_PI_2)); // Orientation 2

    penalty_spots.push_back(FieldFeature(-PENALTY_CROSS_ABS_X, 0.0, 0.0));     // Left Orientation 1
    penalty_spots.push_back(FieldFeature(-PENALTY_CROSS_ABS_X, 0.0, M_PI_2));  // Left Orientation 2
    penalty_spots.push_back(FieldFeature(-PENALTY_CROSS_ABS_X, 0.0, M_PI));    // Left Orientation 3
    penalty_spots.push_back(FieldFeature(-PENALTY_CROSS_ABS_X, 0.0, -M_PI_2)); // Left Orientation 4
    penalty_spots.push_back(FieldFeature(PENALTY_CROSS_ABS_X, 0.0, 0.0));      // Right Orientation 1
    penalty_spots.push_back(FieldFeature(PENALTY_CROSS_ABS_X, 0.0, M_PI_2));   // Right Orientation 2
    penalty_spots.push_back(FieldFeature(PENALTY_CROSS_ABS_X, 0.0, M_PI));     // Right Orientation 3
    penalty_spots.push_back(FieldFeature(PENALTY_CROSS_ABS_X, 0.0, -M_PI_2));  // Right Orientation 4

    // We only use these three lines in the x-direction, because only large lines are passed from field features,
    // and having too many lines in close locations can cause the hypotheses from that observation
    // to merge and get a high weight (because many hypotheses are merged!)
    constantXLines.push_back(-FIELD_LENGTH / 2.0); // Left Goal BaseLine
    constantXLines.push_back(0);                   // centre line
    constantXLines.push_back(FIELD_LENGTH / 2.0);  // Right Goal BaseLine

    // constantXLines.push_back(FIELD_LENGTH / 2.0 - GOAL_BOX_LENGTH);         // Left Goal Box Inside Line
    constantXLines.push_back(FIELD_LENGTH / 2.0 - PENALTY_BOX_LENGTH);         // Left Penalty Box Inside Line
    // constantXLines.push_back(-FIELD_LENGTH / 2.0 + GOAL_BOX_LENGTH);        // Right Goal Box Inside Line
    constantXLines.push_back(-FIELD_LENGTH / 2.0 + PENALTY_BOX_LENGTH);        // Right Penalty Box Inside Line

    constantYLines.push_back(FIELD_WIDTH / 2.0);  // Top Side Line
    constantYLines.push_back(-FIELD_WIDTH / 2.0); // Bottom Side Line

    // constantYLines.push_back(GOAL_BOX_WIDTH / 2.0);     // Left/Right Goal Box Top Line
    // constantYLines.push_back(-GOAL_BOX_WIDTH / 2.0);    // Left/Right Goal Box Bottom Line
    // constantYLines.push_back(-PENALTY_BOX_WIDTH / 2.0); // Left/Right Penalty Box Bottom Line
    // constantYLines.push_back(PENALTY_BOX_WIDTH / 2.0);  // Left/Right Penalty Box Top Line

    edge.x.min = -(FIELD_LENGTH / 2.0 + FIELD_LENGTH_OFFSET);
    edge.x.max =   FIELD_LENGTH / 2.0 + FIELD_LENGTH_OFFSET;
    edge.y.min = -(FIELD_WIDTH / 2.0 + FIELD_WIDTH_OFFSET);
    edge.y.max =   FIELD_WIDTH / 2.0 + FIELD_WIDTH_OFFSET;
}
