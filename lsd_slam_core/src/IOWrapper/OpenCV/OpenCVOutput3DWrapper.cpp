/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "OpenCVOutput3DWrapper.h"
#include "util/settings.h"
#include "util/SophusUtil.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "GlobalMapping/g2oTypeSim3Sophus.h"

#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <stdio.h>


namespace lsd_slam
{


OpenCVOutput3DWrapper::OpenCVOutput3DWrapper(int width, int height)
{
	this->width = width;
	this->height = height;

	publishLvl=0;

	pathFile.open("slamPath.txt", std::ofstream::out);

}

OpenCVOutput3DWrapper::~OpenCVOutput3DWrapper()
{
	pathFile.close();
}


void OpenCVOutput3DWrapper::publishKeyframe(Frame* f)
{
	float camToWorldData[7];
	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();
	memcpy(camToWorldData,f->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	// printf( "Kf pose is: X: %.3f Y: %.3f Z: %.3f\n", camToWorldData[0], camToWorldData[1], camToWorldData[2] );

	// lsd_slam_viewer::keyframeMsg fMsg;


	// boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

	// fMsg.id = f->id();
	// fMsg.time = f->timestamp();
	// fMsg.isKeyframe = true;

	// int w = f->width(publishLvl);
	// int h = f->height(publishLvl);

	// memcpy(fMsg.camToWorld.data(),f->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	// fMsg.fx = f->fx(publishLvl);
	// fMsg.fy = f->fy(publishLvl);
	// fMsg.cx = f->cx(publishLvl);
	// fMsg.cy = f->cy(publishLvl);
	// fMsg.width = w;
	// fMsg.height = h;


	// fMsg.pointcloud.resize(w*h*sizeof(InputPointDense));

	// InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data();

	// const float* idepth = f->idepth(publishLvl);
	// const float* idepthVar = f->idepthVar(publishLvl);
	// const float* color = f->image(publishLvl);

	// for(int idx=0;idx < w*h; idx++)
	// {
	// 	pc[idx].idepth = idepth[idx];
	// 	pc[idx].idepth_var = idepthVar[idx];
	// 	pc[idx].color[0] = color[idx];
	// 	pc[idx].color[1] = color[idx];
	// 	pc[idx].color[2] = color[idx];
	// 	pc[idx].color[3] = color[idx];
	// }

	// keyframe_publisher.publish(fMsg);
}

void OpenCVOutput3DWrapper::publishTrackedFrame(Frame* f)
{
	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();
	Sim3 camToWorld = f->getScaledCamToWorld();
	Eigen::Matrix3d R = camToWorld.rotationMatrix();
	Eigen::Quaterniond q = camToWorld.quaternion();
	double s = camToWorld.scale();
	Eigen::Vector3d t = camToWorld.translation();
	
	pathFile  << t.transpose() << " "
	<< q.coeffs().transpose() << " "
	<< s;

	if (f->pose->trackingParent!=nullptr)
	{
		Sim3 keyframeToWorld = f->pose->trackingParent->getCamToWorld();
		pathFile << " "
		<< keyframeToWorld.translation().transpose() << " "
		<< keyframeToWorld.quaternion().coeffs().transpose() << " "
		<< keyframeToWorld.scale() << " ";

		Sim3 camToKeyframe = f->pose->thisToParent_raw;
		pathFile << " "
		<< camToKeyframe.translation().transpose() << " "
		<< camToKeyframe.quaternion().coeffs().transpose() << " "
		<< camToKeyframe.scale() << " ";

		pathFile << f->timeStampNs();
	}

	pathFile << std::endl;
	
	//pathFile << t.transpose() << " " << q.coeffs().transpose << std::endl;
	

	static unsigned int i = 0;
	if ( i++ % 40 == 0 ) {
		
		printf( "current pose is: X: %.3f Y: %.3f Z: %.3f\n", t(0), t(1), t(2) );
		printf( "qx: %.3f qy: %.3f qz: %.3f qw %.3f\n", q.x(), q.y(), q.z(), q.w() );
		Eigen::Vector3d x(1, 0, 0);
		Eigen::Vector3d y(0, 1, 0);
		Eigen::Vector3d z(0, 0, 1);
		x = q.normalized()._transformVector(x);
		y = q.normalized()._transformVector(y);
		z = q.normalized()._transformVector(z);
		printf( "x axis is: X: %.3f Y: %.3f Z: %.3f\n", x(0), x(1), x(2) );
		printf( "y axis is: X: %.3f Y: %.3f Z: %.3f\n", y(0), y(1), y(2) );
		printf( "z axis is: X: %.3f Y: %.3f Z: %.3f\n", z(0), z(1), z(2) );
		printf( "scale: %.3f\n", s );
	}

	// lsd_slam_viewer::keyframeMsg fMsg;


	// fMsg.id = kf->id();
	// fMsg.time = kf->timestamp();
	// fMsg.isKeyframe = false;


	// memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	// fMsg.fx = kf->fx(publishLvl);
	// fMsg.fy = kf->fy(publishLvl);
	// fMsg.cx = kf->cx(publishLvl);
	// fMsg.cy = kf->cy(publishLvl);
	// fMsg.width = kf->width(publishLvl);
	// fMsg.height = kf->height(publishLvl);

	// fMsg.pointcloud.clear();

	// liveframe_publisher.publish(fMsg);


	// SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());

	// geometry_msgs::PoseStamped pMsg;

	// pMsg.pose.position.x = camToWorld.translation()[0];
	// pMsg.pose.position.y = camToWorld.translation()[1];
	// pMsg.pose.position.z = camToWorld.translation()[2];
	// pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
	// pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
	// pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
	// pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

	// if (pMsg.pose.orientation.w < 0)
	// {
	// 	pMsg.pose.orientation.x *= -1;
	// 	pMsg.pose.orientation.y *= -1;
	// 	pMsg.pose.orientation.z *= -1;
	// 	pMsg.pose.orientation.w *= -1;
	// }

	// pMsg.header.stamp = ros::Time(kf->timestamp());
	// pMsg.header.frame_id = "world";
	// pose_publisher.publish(pMsg);
}



void OpenCVOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{
	// lsd_slam_viewer::keyframeGraphMsg gMsg;

	// graph->edgesListsMutex.lock();
	// gMsg.numConstraints = graph->edgesAll.size();
	// gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
	// GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data();
	// for(unsigned int i=0;i<graph->edgesAll.size();i++)
	// {
	// 	constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
	// 	constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
	// 	Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
	// 	constraintData[i].err = sqrt(err.dot(err));
	// }
	// graph->edgesListsMutex.unlock();

	// graph->keyframesAllMutex.lock_shared();
	// gMsg.numFrames = graph->keyframesAll.size();
	// gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
	// GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
	// for(unsigned int i=0;i<graph->keyframesAll.size();i++)
	// {
	// 	framePoseData[i].id = graph->keyframesAll[i]->id();
	// 	memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
	// }
	// graph->keyframesAllMutex.unlock_shared();

	// graph_publisher.publish(gMsg);
}

void OpenCVOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void OpenCVOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void OpenCVOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
	// std_msgs::Float32MultiArray msg;
	// for(int i=0;i<20;i++)
	// 	msg.data.push_back((float)(data[i]));

	// debugInfo_publisher.publish(msg);
}

}
