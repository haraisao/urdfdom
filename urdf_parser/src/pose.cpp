/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Wim Meeussen, John Hsu */


#include <urdf_model/pose.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <console_bridge/console.h>
#ifndef WIN32
#include <tinyxml.h>
#else
#include <tinyxml2.h>
#define TiXmlDocument tinyxml2::XMLDocument
#define TiXmlElement tinyxml2::XMLElement
#endif
#include <urdf_parser/urdf_parser.h>

namespace urdf_export_helpers {
#ifndef WIN32
std::string values2str(unsigned int count, const double *values, double (*conv)(double))
#else
const char * values2str(unsigned int count, const double *values, double (*conv)(double))
#endif
{
    std::stringstream ss;
    for (unsigned int i = 0 ; i < count ; i++)
    {
        if (i > 0)
            ss << " ";
        ss << (conv ? conv(values[i]) : values[i]);
    }
#ifndef WIN32
    return ss.str();
#else
    return ss.str().c_str();
#endif
}
#ifndef WIN32
std::string values2str(urdf::Vector3 vec)
#else
const char* values2str(urdf::Vector3 vec)
#endif
{
    double xyz[3];
    xyz[0] = vec.x;
    xyz[1] = vec.y;
    xyz[2] = vec.z;
    return values2str(3, xyz);
}
#ifndef WIN32
std::string values2str(urdf::Rotation rot)
#else
const char * values2str(urdf::Rotation rot)
#endif
{
    double rpy[3];
    rot.getRPY(rpy[0], rpy[1], rpy[2]);
    return values2str(3, rpy);
}
#ifndef WIN32
std::string values2str(urdf::Color c)
#else
const char * values2str(urdf::Color c)
#endif
{
    double rgba[4];
    rgba[0] = c.r;
    rgba[1] = c.g;
    rgba[2] = c.b;
    rgba[3] = c.a;
    return values2str(4, rgba);
}
#ifndef WIN32
std::string values2str(double d)
#else
const char * values2str(double d)
#endif
{
    return values2str(1, &d);
}
}

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml)
{
  pose.clear();
  if (xml)
  {
    const char* xyz_str = xml->Attribute("xyz");
    if (xyz_str != NULL)
    {
      try {
        pose.position.init(xyz_str);
      }
      catch (ParseError &e) {
        CONSOLE_BRIDGE_logError(e.what());
        return false;
      }
    }

    const char* rpy_str = xml->Attribute("rpy");
    if (rpy_str != NULL)
    {
      try {
        pose.rotation.init(rpy_str);
      }
      catch (ParseError &e) {
        CONSOLE_BRIDGE_logError(e.what());
        return false;
      }
    }
  }
  return true;
}

bool exportPose(Pose &pose, TiXmlElement* xml)
{
#ifndef WIN32
  TiXmlElement *origin = new TiXmlElement("origin");
#else
  tinyxml2::XMLElement *origin = xml->GetDocument()->NewElement("origin");
#endif
  std::string pose_xyz_str = urdf_export_helpers::values2str(pose.position);
  std::string pose_rpy_str = urdf_export_helpers::values2str(pose.rotation);
#ifndef WIN32
  origin->SetAttribute("xyz", pose_xyz_str);
  origin->SetAttribute("rpy", pose_rpy_str);
#else
  origin->SetAttribute("xyz", pose_xyz_str.c_str());
  origin->SetAttribute("rpy", pose_rpy_str.c_str());
#endif
  xml->LinkEndChild(origin);
  return true;
}

}


