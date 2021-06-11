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
  }

  std::string host = argv[1];
  int port = std::stoi(argv[2]);
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
  head->start();

  auto camera = std::make_shared<CameraMeasurement>();
  cv::Mat frame, frame_hsv, field_mask;
  keisan::Point2 ball_pos;
  float view_h_angle, view_v_angle;
  // float ball_position_x, ball_position_y;
  // std::uint32_t center_x, center_y;
  // double pan = 0.0, tilt = 0.0;
  ninshiki_opencv::Detector detector;

  while (client.get_tcp_socket()->is_connected()) {
    try {
      client.send(*message.get_actuator_request());
      auto sensors = client.receive();

      // Set current pan and current tilt
      if (sensors.get()->position_sensors_size() > 0) {
        double pan = sensors.get()->position_sensors(0).value();
        double tilt = sensors.get()->position_sensors(1).value();
        head->set_pan_tilt_angle(pan, tilt);
        head->initialize();
        head->load_data();
      }

      // Get Ball Position
      if (sensors.get()->cameras_size() > 0) {
        *camera = sensors.get()->cameras(0);
        cv::Mat temp = detector.get_image(sensors);

        frame = temp.clone();
        frame_hsv = temp.clone();
        cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

        detector.vision_process(frame_hsv, frame);

        ball_pos = keisan::Point2(detector.get_ball_pos_x(), detector.get_ball_pos_y());
      }

      std::cout << "ball_position_x = " << ball_pos.x << std::endl;
      std::cout << "ball_position_y = " << ball_pos.y << std::endl;

      if (ball_pos.x == 0 && ball_pos.y == 0) {
        std::cout << "ball is lost" << std::endl;
        head->move_scan_ball_down();
      } else if (ball_pos.x != 0 || ball_pos.y != 0) {
        std::cout << "track ball" << std::endl;
        head->track_ball(head, camera, ball_pos);
      }
      head->process();
      std::cout << "pan = " << head->get_pan_angle() << " tilt = " << head->get_tilt_angle() <<
        std::endl;
      message.clear_actuator_request();
      for (auto joint : head->get_joints()) {
        message.add_motor_position_in_degree(joint.get_joint_name(), joint.get_goal_position());
        std::cout << joint.get_joint_name() << " " << joint.get_goal_position() << std::endl;
      }
      client.send(*message.get_actuator_request());
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}
