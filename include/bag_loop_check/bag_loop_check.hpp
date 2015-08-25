/* Bag Loop Check Utility Class
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2014-2015 Michael 'v4hn' Goerner
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#ifndef _BAG_LOOP_CHECK_HPP_
#define _BAG_LOOP_CHECK_HPP_

#include <ros/time.h>

namespace bag_loop_check {

	/* BagLoopCheck - correctly detect bag loops
	 *
	 * Example Usage:
	 *
	 * void Class:callback(const Message::ConstPtr& msg){
	 * static bag_loop_check::BagLoopCheck bag_loop;
	 * if(bag_loop){
	 *   ROS_WARN("bag loop detected. Resetting Class");
	 *   // clean up your data structures
	 * }
	 */
	class BagLoopCheck {
	public:
		BagLoopCheck(ros::Duration tolerance= ros::Duration(1)) :
			active_(ros::NodeHandle().param("use_sim_time", false)),
			last_chk_(0),
			tolerance_(tolerance)
		{}

		operator bool(){
			if(active_){
				ros::Time now= ros::Time::now();
				bool backjump= now - last_chk_ < -tolerance_;
				last_chk_= now;
				return backjump;
			}
			return false;
		}

		bool isActive(){
			return active_;
		}

	private:
		bool active_;
		ros::Time last_chk_;
		ros::Duration tolerance_;
	};

}
#endif
