
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sebastian Jahr */

#include <moveit/planning_pipeline_interfaces/plan_responses_container.hpp>

namespace moveit
{
namespace planning_pipeline_interfaces
{
PlanResponsesContainer::PlanResponsesContainer(const size_t expected_size)
{
  solutions_.reserve(expected_size);
}

void PlanResponsesContainer::pushBack(const MotionPlanRequestAndResponse& plan_solution)
{
  std::lock_guard<std::mutex> lock_guard(solutions_mutex_);
  solutions_.push_back(plan_solution);
}

const std::vector<::planning_interface::MotionPlanResponse> PlanResponsesContainer::getSolutions() const
{
  // Otherwise, just return the unordered list of solutions
  std::vector<::planning_interface::MotionPlanResponse> output;
  output.reserve(solutions_.size());  // 预分配空间以提高效率

  for (const auto& tuple : solutions_)
  {
    // 提取元组的第二个元素（索引1）
    output.push_back(std::get<1>(tuple));
  }
  return output;
}

const std::vector<MotionPlanRequestAndResponse>& PlanResponsesContainer::getRequestAndSolutions() const
{
  return solutions_;
}
}  // namespace planning_pipeline_interfaces
}  // namespace moveit
