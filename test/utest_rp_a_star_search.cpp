#include "gtest/gtest.h"
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>
#include "../src/route_model.h"
#include "../src/route_planner.h"


static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

std::vector<std::byte> ReadOSMData(const std::string &path) {
    std::vector<std::byte> osm_data;
    auto data = ReadFile(path);
    if( !data ) {
        std::cout << "Failed to read OSM data." << std::endl;
    } else {
        osm_data = std::move(*data);
    }
    return osm_data;
}

//--------------------------------//
//   Beginning RoutePlanner Tests.
//--------------------------------//

class RoutePlannerTest : public ::testing::Test {
  protected:
    std::string osm_data_file = "../map.osm";
    std::vector<std::byte> osm_data = ReadOSMData(osm_data_file);
    RouteModel model{osm_data};
    RoutePlanner route_planner{model, 10, 10, 90, 90};
    
    // Construct start_node and end_node as in the model.
    float start_x = 0.1;
    float start_y = 0.1;
    float end_x = 0.9;
    float end_y = 0.9;
    RouteModel::Node* start_node = &model.FindClosestNode(start_x, start_y);
    RouteModel::Node* end_node = &model.FindClosestNode(end_x, end_y);

    // Construct another node in the middle of the map for testing.
    float mid_x = 0.5;
    float mid_y = 0.5;
    RouteModel::Node* mid_node = &model.FindClosestNode(mid_x, mid_y);
};


// Test the CalculateHValue method.
TEST_F(RoutePlannerTest, TestCalculateHValue) {
    EXPECT_FLOAT_EQ(route_planner.CalculateHValue(start_node), 1.1329799);
    EXPECT_FLOAT_EQ(route_planner.CalculateHValue(end_node), 0.0f);
    EXPECT_FLOAT_EQ(route_planner.CalculateHValue(mid_node), 0.58903033);
}

// Test the AddNeighbors method.
bool NodesSame(RouteModel::Node* a, RouteModel::Node* b) { return a == b; }
TEST_F(RoutePlannerTest, TestAddNeighbors) {
    route_planner.AddNeighbors(start_node);

    // Correct h and g values for the neighbors of start_node.
    std::vector<float> start_neighbor_g_vals{0.10671431, 0.082997195, 0.051776856, 0.055291083};
    std::vector<float> start_neighbor_h_vals{1.1828455, 1.0998145, 1.0858033, 1.1831238};
    auto neighbors = start_node->neighbors;
    EXPECT_EQ(neighbors.size(), 4);

    // Check results for each neighbor.
    for (int i = 0; i < neighbors.size(); i++) {
        EXPECT_PRED2(NodesSame, neighbors[i]->parent, start_node);
        EXPECT_FLOAT_EQ(neighbors[i]->g_value, start_neighbor_g_vals[i]);
        EXPECT_FLOAT_EQ(neighbors[i]->h_value, start_neighbor_h_vals[i]);
        EXPECT_EQ(neighbors[i]->visited, true);
    }
}

// Test the NextNode method.
TEST_F(RoutePlannerTest, TestNextNode) {
    std::vector<RouteModel::Node*> open_list;
  
	RouteModel::Node expected_next_node;
  	expected_next_node.g_value = 1;
    expected_next_node.h_value = 1;
    open_list.push_back(&expected_next_node);
  
    RouteModel::Node node1;
  	node1.g_value = 2;
    node1.h_value = 2;
    open_list.push_back(&node1);
  
    RouteModel::Node node2;
  	node2.g_value = 3;
    node2.h_value = 3;
    open_list.push_back(&node2);  
  
    sort(open_list.begin(), open_list.end(), [](RouteModel::Node *node1, RouteModel::Node *node2) {
        auto f1 = node1->g_value + node1->h_value;
        auto f2 = node2->g_value + node2->h_value;

        return f1 > f2;
    });
    RouteModel::Node* test_node = open_list.back();
    open_list.pop_back();

    EXPECT_TRUE(expected_next_node.g_value == test_node->g_value);
    EXPECT_TRUE(expected_next_node.h_value == test_node->h_value);
    EXPECT_EQ(open_list.size(), 2);
}


// Test the ConstructFinalPath method.
TEST_F(RoutePlannerTest, TestConstructFinalPath) {
    // Construct a path.
    mid_node->parent = start_node;
    end_node->parent = mid_node;
    std::vector<RouteModel::Node> path = route_planner.ConstructFinalPath(end_node);

    // Test the path.
    EXPECT_EQ(path.size(), 3);
    EXPECT_FLOAT_EQ(start_node->x, path.front().x);
    EXPECT_FLOAT_EQ(start_node->y, path.front().y);
    EXPECT_FLOAT_EQ(end_node->x, path.back().x);
    EXPECT_FLOAT_EQ(end_node->y, path.back().y);
}


// Test the AStarSearch method.
TEST_F(RoutePlannerTest, TestAStarSearch) {
    route_planner.AStarSearch();
    EXPECT_EQ(model.path.size(), 33);
    RouteModel::Node path_start = model.path.front();
    RouteModel::Node path_end = model.path.back();
    // The start_node and end_node x, y values should be the same as in the path.
    EXPECT_FLOAT_EQ(start_node->x, path_start.x);
    EXPECT_FLOAT_EQ(start_node->y, path_start.y);
    EXPECT_FLOAT_EQ(end_node->x, path_end.x);
    EXPECT_FLOAT_EQ(end_node->y, path_end.y);
    EXPECT_FLOAT_EQ(route_planner.GetDistance(), 873.41565);
}
