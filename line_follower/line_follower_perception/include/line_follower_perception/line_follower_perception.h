// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _LINE_FOLLOWER_PERCEPTION_H_
#define _LINE_FOLLOWER_PERCEPTION_H_

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/dnn_node_data.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::ModelTaskType;

using hobot::dnn_node::DNNTensor;

/**
 * @brief 线条坐标结果类
 * 存储从神经网络输出解析出的线条中心点坐标
 */
class LineCoordinateResult {
 public:
  float x;  // 线条中心点x坐标
  float y;  // 线条中心点y坐标
  
  /**
   * @brief 重置坐标值为默认值
   * 将x和y坐标重置为-1.0，表示未检测到有效线条
   */
  void Reset() {x = -1.0; y = -1.0;}
};

/**
 * @brief 线条坐标解析器类
 * 负责从神经网络输出的张量中解析出线条坐标
 */
class LineCoordinateParser {
 public:
  LineCoordinateParser() {}  // 构造函数
  ~LineCoordinateParser() {} // 析构函数
  
  /**
   * @brief 解析神经网络输出张量，提取线条坐标
   * @param[out] output 存储解析结果的LineCoordinateResult指针
   * @param[in] output_tensor 神经网络输出的张量
   * @return 解析结果状态码，0表示成功
   */
  int32_t Parse(
      std::shared_ptr<LineCoordinateResult>& output,
      std::shared_ptr<DNNTensor>& output_tensor);
};

/**
 * @brief 线跟踪感知节点类
 * 主要的ROS2节点，负责接收图像数据，进行神经网络推理，
 * 解析线条坐标，并发布控制命令
 */
class LineFollowerPerceptionNode : public DnnNode {
 public:
  /**
   * @brief 构造函数
   * @param node_name 节点名称
   * @param options ROS2节点选项
   */
  LineFollowerPerceptionNode(const std::string& node_name,
                        const NodeOptions &options = NodeOptions());
  
  /**
   * @brief 析构函数
   */
  ~LineFollowerPerceptionNode() override;

 protected:
  /**
   * @brief 设置节点参数
   * 重写父类方法，用于配置DNN节点的参数
   * @return 设置结果状态码，0表示成功
   */
  int SetNodePara() override;
  
  /**
   * @brief 后处理函数
   * 重写父类方法，用于处理神经网络推理结果
   * @param outputs 神经网络输出结果
   * @return 处理结果状态码，0表示成功
   */
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  /**
   * @brief 预测函数
   * 调用DNN节点的Run方法进行推理
   * @param dnn_inputs 神经网络输入数据
   * @param output 存储输出结果的指针
   * @param rois 感兴趣区域
   * @return 预测结果状态码
   */
  int Predict(std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
              const std::shared_ptr<DnnNodeOutput> &output,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois);
  
  /**
   * @brief 订阅者回调函数
   * 处理接收到的图像消息
   * @param msg HBM图像消息
   */
  void subscription_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg);
  
  /**
   * @brief 获取参数
   * 从ROS2参数服务器获取配置参数
   * @return 获取结果，true表示成功
   */
  bool GetParams();
  
  /**
   * @brief 赋值参数
   * 将获取到的参数赋值给成员变量
   * @param parameters 参数列表
   * @return 赋值结果，true表示成功
   */
  bool AssignParams(const std::vector<rclcpp::Parameter> & parameters);
  
  ModelTaskType model_task_type_;  // 模型任务类型，默认为模型推理类型
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr subscriber_;  // 图像订阅者
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  // 控制命令发布者
  cv::Mat image_bgr_;  // BGR格式图像
  std::string model_path_;  // 模型路径
  std::string model_name_;  // 模型名称
};

#endif  // _LINE_FOLLOWER_PERCEPTION_H_