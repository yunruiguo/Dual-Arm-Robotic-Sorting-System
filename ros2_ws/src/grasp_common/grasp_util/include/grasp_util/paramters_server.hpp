// Copyright (c) 2012, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GRASP_UTIL__PARAMTERS_SERVER_HPP_
#define GRASP_UTIL__PARAMTERS_SERVER_HPP_

#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

namespace grasp_util {

template <typename T>
struct identity {
  typedef T type;
};

template <typename T>
void declare_param(rclcpp::Node* node, const std::string param_name, T& param,
                   const typename identity<T>::type& default_value) {
  node->declare_parameter(param_name, default_value);
  node->get_parameter(param_name, param);
  RCLCPP_INFO_STREAM(node->get_logger(), param_name << " = " << param);
}

template <typename T>
void declare_param_vector(rclcpp::Node* node, const std::string param_name,
                          T& param,
                          const typename identity<T>::type& default_value) {
  node->declare_parameter(param_name, default_value);
  node->get_parameter(param_name, param);
  for (size_t i = 0; i < param.size(); i++) {
    RCLCPP_INFO_STREAM(node->get_logger(), param_name << "[" << i << "]"
                                                      << " = " << param[i]);
  }
}

}  // end namespace grasp_util

#endif  // GRASP_UTIL__PARAMTERS_SERVER_HPP_
