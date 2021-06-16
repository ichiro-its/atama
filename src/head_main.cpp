// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <aruku/walking.hpp>
#include <atama/head.hpp>
#include <kansei/imu.hpp>
#include <ninshiki_opencv/detector.hpp>
#include <robocup_client/robocup_client.hpp>

#include <memory>
#include <string>

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::cerr << "Please specify the host and the port!" << std::endl;
    return 0;
  } else if (argc < 4) {
    std::cerr << "Please specify the ball_cascade.xml location!" << std::endl;
    return 0;
  }

  std::string host = argv[1];
  int port = std::stoi(argv[2]);
  std::string path = argv[3];
  robocup_client::RobotClient client(host, port);
  if (!client.connect()) {
    std::cerr << "Failed to connect to server on port " << client.get_port() << "!" << std::endl;
    return 1;
  }

  robocup_client::MessageHandler message;
  message.add_sensor_time_step("Camera", 16);
  message.add_sensor_time_step("neck_yaw_s", 8);
  message.add_sensor_time_step("neck_pitch_s", 8);
  client.send(*message.get_actuator_request());

  auto imu = std::make_shared<kansei::Imu>();

  auto walking = std::make_shared<aruku::Walking>(imu);

  auto head = std::make_shared<atama::Head>(walking, imu);
  head->initialize();

  auto camera = std::make_shared<CameraMeasurement>();
  cv::Mat frame, frame_hsv, field_mask;
  keisan::Point2 ball_pos;
  float view_h_angle, view_v_angle;

  ninshiki_opencv::Detector detector;

  while (client.get_tcp_socket()->is_connected()) {
    try {
      client.send(*message.get_actuator_request());
      auto sensors = client.receive();

      // Load data
      head->load_data(path);

      // Get Ball Position
      if (sensors.get()->cameras_size() > 0) {
        *camera = sensors.get()->cameras(0);
        cv::Mat temp = detector.get_image(sensors);

        frame = temp.clone();
        frame_hsv = temp.clone();
        cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

        detector.vision_process(frame_hsv, frame);

        float diagonal = alg::distance(camera->width(), camera->height());
        int field_of_view = 78;
        float depth = (diagonal / 2) / tan(field_of_view * alg::deg2Rad() / 2);

        view_h_angle = 2 * atan2(camera->width() / 2, depth) * alg::rad2Deg();
        view_v_angle = 2 * atan2(camera->height() / 2, depth) * alg::rad2Deg();

        ball_pos = keisan::Point2(detector.get_ball_pos_x(), detector.get_ball_pos_y());
      }

      if (ball_pos.x == 0 && ball_pos.y == 0) {
        std::cout << "ball is lost" << std::endl;
        head->move_scan_ball_down();
      } else if (ball_pos.x != 0 || ball_pos.y != 0) {
        std::cout << "track ball" << std::endl;
        head->track_ball(camera, ball_pos, view_v_angle, view_h_angle);
      }
      head->process();
      message.clear_actuator_request();
      for (auto joint : head->get_joints()) {
        message.add_motor_position_in_degree(joint.get_joint_name(), joint.get_goal_position());
      }
      client.send(*message.get_actuator_request());
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}
