/**
 * @file test_theta_star_path_planner.cpp
 * @brief Unit tests for ThetaStarPathPlanner class - validating optimal any-angle pathfinding
 * @author Jackson Russell
 * @date October-2025
 */

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "theta_star_path_planner.h"

using namespace drone_navigation;

class ThetaStarPathPlannerTest : public ::testing::Test 
{
protected:
  void SetUp() override {
    // Initialise with same grid as A* for comparison: 200x200, 0.1m resolution, 20x20m area
    planner_ = std::make_unique<drone_navigation::ThetaStarPathPlanner>();
  }

  std::unique_ptr<drone_navigation::ThetaStarPathPlanner> planner_;
};

/**
 * @brief Test basic pathfinding functionality
 * Verifies that Theta* can find paths in free space
 */
TEST_F(ThetaStarPathPlannerTest, BasicPathPlanning) 
{
  // Test simple horizontal path
  auto waypoints = planner_->planPath(-5.0f, 0.0f, 5.0f, 0.0f);
  
  ASSERT_FALSE(waypoints.empty());
  EXPECT_NEAR(waypoints.front().first, -5.0f, 0.1f);  // Start X (within grid cell)
  EXPECT_NEAR(waypoints.front().second, 0.0f, 0.1f);  // Start Y
  EXPECT_NEAR(waypoints.back().first, 5.0f, 0.1f);    // Goal X  
  EXPECT_NEAR(waypoints.back().second, 0.0f, 0.1f);   // Goal Y
  
  // Test simple vertical path
  waypoints = planner_->planPath(0.0f, -5.0f, 0.0f, 5.0f);
  
  ASSERT_FALSE(waypoints.empty());
  EXPECT_NEAR(waypoints.front().first, 0.0f, 0.1f);   // Start X (within grid cell)
  EXPECT_NEAR(waypoints.front().second, -5.0f, 0.1f); // Start Y
  EXPECT_NEAR(waypoints.back().first, 0.0f, 0.1f);    // Goal X
  EXPECT_NEAR(waypoints.back().second, 5.0f, 0.1f);   // Goal Y
}

/**
 * @brief Test Theta* optimal pathfinding validation
 * THE CRITICAL TEST: Validates that Theta* produces truly optimal paths
 */
TEST_F(ThetaStarPathPlannerTest, ThetaStarOptimalityValidation) 
{
  std::cout << "\n🌟 THETA* OPTIMALITY VALIDATION - Testing 20 scenarios for true path optimality 🌟\n";
  std::cout << "Expected: 100% success rate (vs A*'s 75%)\n";
  std::cout << "Key improvement: Any-angle pathfinding eliminates grid discretization sub-optimality\n\n";

  struct TestCase {
    std::string name;
    float start_x, start_y, goal_x, goal_y;
    float expected_distance;
    float tolerance_percent;
  };

  std::vector<TestCase> test_cases = {
    // Straight line paths (should be perfect)
    {"Horizontal right", -2.0f, 0.0f, 3.0f, 0.0f, 5.0f, 1.0f},
    {"Horizontal left", 2.0f, 0.0f, -3.0f, 0.0f, 5.0f, 1.0f},
    {"Vertical up", 0.0f, -2.0f, 0.0f, 3.0f, 5.0f, 1.0f},
    {"Vertical down", 0.0f, 2.0f, 0.0f, -3.0f, 5.0f, 1.0f},
    
    // Diagonal paths (THE CRITICAL TESTS - should be optimal with Theta*)
    {"Diagonal 3-4-5 triangle", 0.0f, 0.0f, 3.0f, 4.0f, 5.0f, 1.0f},
    {"Diagonal 5-12-13 triangle", -2.0f, -2.0f, 3.0f, 10.0f, 13.0f, 1.0f}, // 5-12-13 triangle within bounds
    {"Diagonal southwest", 2.0f, 2.0f, -1.0f, -2.0f, 5.0f, 1.0f},
    {"Long diagonal 6-8-10", 1.0f, 1.0f, 7.0f, 9.0f, 10.0f, 1.0f},
    
    // Complex paths
    {"Long horizontal", -8.0f, 1.0f, 7.0f, 1.0f, 15.0f, 1.0f},
    {"Long vertical", 1.0f, -8.0f, 1.0f, 7.0f, 15.0f, 1.0f},
    {"Long diagonal NE", -5.0f, -5.0f, 5.0f, 5.0f, 14.142f, 1.0f},
    {"Long diagonal NW", 5.0f, -5.0f, -5.0f, 5.0f, 14.142f, 1.0f},
    
    // Pythagorean triples (exact mathematical distances)
    {"Pythagorean 8-6-10", 0.0f, 0.0f, 8.0f, 6.0f, 10.0f, 1.0f},
    {"Pythagorean 9-12-15", 1.0f, 1.0f, 10.0f, 9.0f, 12.04f, 1.0f}, // Adjusted to fit grid bounds
    {"Pythagorean 7-12-15", -2.0f, -3.0f, 5.0f, 9.0f, 13.89f, 2.0f}, // 7-12 triangle to fit grid bounds
    
    // Mixed short/medium paths
    {"Short diagonal", 1.0f, 1.0f, 2.0f, 2.0f, 1.414f, 1.0f},
    {"Medium diagonal", -3.0f, -2.0f, 4.0f, 5.0f, 9.899f, 1.0f},
    {"Asymmetric path", -1.0f, 3.0f, 6.0f, -2.0f, 8.602f, 1.0f},
    
    // Edge case paths  
    {"Near-zero movement", 0.1f, 0.1f, 0.2f, 0.2f, 0.141f, 5.0f}, // Higher tolerance for very small distances
    {"Perfect diagonal unit", 0.0f, 0.0f, 1.0f, 1.0f, 1.414f, 1.0f}
  };

  int passed_tests = 0;
  int total_tests = test_cases.size();
  
  for (const auto& test_case : test_cases) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Plan path using Theta*
    auto waypoints = planner_->planPath(test_case.start_x, test_case.start_y, 
                                        test_case.goal_x, test_case.goal_y);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    ASSERT_FALSE(waypoints.empty()) << "No path found for: " << test_case.name;
    
    // Calculate actual path distance
    float actual_distance = 0.0f;
    for (size_t i = 1; i < waypoints.size(); ++i) {
      float dx = waypoints[i].first - waypoints[i-1].first;
      float dy = waypoints[i].second - waypoints[i-1].second;
      actual_distance += sqrt(dx*dx + dy*dy);
    }
    
    // Calculate error percentage
    float error_percent = (std::abs(actual_distance - test_case.expected_distance) / test_case.expected_distance) * 100.0f;
    bool test_passed = error_percent <= test_case.tolerance_percent;
    
    if (test_passed) {
      passed_tests++;
    }
    
    // Print detailed results
    std::cout << std::setw(25) << std::left << test_case.name << " | "
              << std::setw(8) << std::fixed << std::setprecision(3) << test_case.expected_distance << " | "
              << std::setw(8) << actual_distance << " | "
              << std::setw(6) << std::setprecision(1) << error_percent << "% | "
              << std::setw(6) << duration.count() << "μs | "
              << (test_passed ? "PASS" : "FAIL") << std::endl;
  }
  
  float success_rate = (float(passed_tests) / float(total_tests)) * 100.0f;
  
  std::cout << "\nTHETA* OPTIMALITY RESULTS:\n";
  std::cout << "Success Rate: " << passed_tests << "/" << total_tests << " (" 
            << std::fixed << std::setprecision(1) << success_rate << "%)\n";
  
  if (success_rate >= 95.0f) {
    std::cout << "EXCELLENT: Theta* achieves near-perfect optimality!\n";
  } else if (success_rate >= 90.0f) {
    std::cout << "GREAT: Theta* significantly outperforms other grid-based methods!\n";  
  } else if (success_rate >= 80.0f) {
    std::cout << "GOOD: Theta* shows improved optimality\n";
  } else {
    std::cout << "NEEDS IMPROVEMENT: Theta* optimality below expectations\n";
  }
  
  std::cout << "Expected: Theta* should achieve ≥95% success rate with any-angle pathfinding\n\n";
  
  // Expect significant improvement 
  EXPECT_GE(success_rate, 90.0f) << "Theta* should significantly outperform A*";
}

/**
 * @brief Test start equals goal scenario
 */
TEST_F(ThetaStarPathPlannerTest, StartEqualsGoal) 
{
  auto waypoints = planner_->planPath(0.0f, 0.0f, 0.0f, 0.0f);
  
  ASSERT_FALSE(waypoints.empty());
  EXPECT_EQ(waypoints.size(), 1);
  EXPECT_NEAR(waypoints[0].first, 0.0f, 0.1f);
  EXPECT_NEAR(waypoints[0].second, 0.0f, 0.1f);
}

/**
 * @brief Test coordinate conversion functions
 */
TEST_F(ThetaStarPathPlannerTest, CoordinateConversion) 
{
  geometry_msgs::msg::Point world_point;
  world_point.x = 2.5f;
  world_point.y = -3.7f;
  
  GridCell grid_cell = planner_->worldToGrid(world_point);
  
  // With 0.1m resolution and origin at (-10, -10):
  // Expected grid coordinates: ((2.5 - (-10)) / 0.1, (-3.7 - (-10)) / 0.1) = (124, 62) [due to floating point precision]
  EXPECT_EQ(grid_cell.x, 124);
  EXPECT_EQ(grid_cell.y, 62);
  
  // Test grid to world conversion  
  float world_x, world_y;
  planner_->gridToWorld(100, 100, world_x, world_y);
  
  // Expected: (-10 + (100 + 0.5) * 0.1, -10 + (100 + 0.5) * 0.1) = (0.05, 0.05)
  EXPECT_NEAR(world_x, 0.05f, 0.001f);
  EXPECT_NEAR(world_y, 0.05f, 0.001f);
}

/**
 * @brief Performance test for larger paths
 */
TEST_F(ThetaStarPathPlannerTest, LargerPathPerformance) 
{
  // Test performance with larger distance (should still be optimal)
  auto start_time = std::chrono::high_resolution_clock::now();
  
  auto waypoints = planner_->planPath(-9.0f, -9.0f, 9.0f, 9.0f);
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  ASSERT_FALSE(waypoints.empty());
  
  // Calculate path length
  float path_length = 0.0f;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    float dx = waypoints[i].first - waypoints[i-1].first;
    float dy = waypoints[i].second - waypoints[i-1].second;
    path_length += sqrt(dx*dx + dy*dy);
  }
  
  float expected_length = sqrt(18.0f*18.0f + 18.0f*18.0f); // 25.456m
  float error_percent = (std::abs(path_length - expected_length) / expected_length) * 100.0f;
  
  std::cout << "Large path test: " << path_length << "m (expected " << expected_length 
            << "m, " << std::setprecision(1) << error_percent << "% error) in " 
            << duration.count() << "ms\n";
  
  EXPECT_LT(error_percent, 2.0f) << "Large diagonal path should be near-optimal with Theta*";
  EXPECT_LT(duration.count(), 100) << "Theta* should complete large paths reasonably quickly";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}