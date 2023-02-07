#include <gtest/gtest.h>
#include <iostream>
#include "dewarp.cpp"



template<class T = double>
void test_match_scan(vector<LineSegment<T>>&world, Pose<T>&pose_1, Pose<T>&pose_2, bool log_goal_steps = false) {
    size_t n_points = 360;

    auto scan1 = scan_with_twist<T>(world, n_points, 0, 0, 0, pose_1);
    auto scan2 = scan_with_twist<T>(world, n_points, 0, 0, 0, pose_2);
    auto guess = pose_1;

    ScanMatch<T> matched_pose = match_scans<T>(scan1, scan2, guess, log_goal_steps);


    cout << "actual pose: " << to_string(pose_2)
         << " -> matched pose: " << to_string(matched_pose.delta) 
         << " error_x: " << fixed << setprecision(3) << matched_pose.delta.get_x() - pose_2.get_x()
         << " error_y: " << matched_pose.delta.get_y() - pose_2.get_y()
         << " error_theta: " << setprecision(1) << radians2degrees(matched_pose.delta.get_theta() - pose_2.get_theta())
         << " score: " << matched_pose.score
         << endl;
}

template<class T = double>
void test_match_scans() {
    auto world = get_world<T>();

    std::normal_distribution<T> x_value(0.0,0.1);
    std::normal_distribution<T> y_value(0.0,0.1);
    std::normal_distribution<T> theta_value(0.0,degrees2radians(3));

    Pose<T> pose_1;
    Pose<T> pose_2(x_value(random_engine), y_value(random_engine), (theta_value(random_engine)));

    test_match_scan<T>(world, pose_1, pose_2, false);

}

template <class T>
void test_match_n_scans(size_t n) {
    cout << "matching  " << n << " random scans" << endl;
    for(size_t i = 0; i < n; ++i) test_match_scans<T>();
    cout << "done matching  " << n << " random scans" << endl;
}



TEST(dewarp, hand_picked_match_scans) {
  auto world = get_world<float>();

  Pose<float> origin;
  Pose<float> p(-0.08, -0.143, degrees2radians(-3.3));

  test_match_scan<float>(world, origin, p, false);
}



TEST(dewarp, random_match_scans) {
    test_match_n_scans<float>(100);
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
//  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
//  rclcpp::shutdown();

  return test_result;
}