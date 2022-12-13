#include "create_restrict2.h"
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include "rviz/display_context.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz_custom::Restrict2, rviz::Tool)

namespace rviz_custom {

Restrict2::Restrict2() {

    Size_property_2 = new rviz::IntProperty("Zone_num", 1,"Select of zone number",
                                          getPropertyContainer(), SLOT(updateTopic()), this);

    Remove_all_2 = new rviz::BoolProperty("Clear", false, "State of Clear or Draw ",
                                         getPropertyContainer(), SLOT(updateTopic()), this);

    float_property = new rviz::FloatProperty("Speed", 0.0,"Select of zone speed ",
                                          getPropertyContainer(), SLOT(updateTopic()), this);

}

Restrict2::~Restrict2() {}

void Restrict2::updateTopic()
{
    settings_.x = Size_property_2->getInt();
    settings_.y = Remove_all_2->getBool(); //clear
    settings_.z = float_property->getFloat(); //speed value

}

void Restrict2::initialize()
{
    pub_  = n_.advertise<geometry_msgs::Point>("/mouse_location2",1);
    pub2_ = n_.advertise<geometry_msgs::Point>("/restrict_settings2",1);
    pub3_ = n_.advertise<geometry_msgs::Point>("/zone_rectangle_data",1);

}
void Restrict2::activate()
{
    initialize();
    ROS_INFO("Initialize is starting2!");
    operation_=false;
    set_rectangle_.z = 0.0;
}

void Restrict2::deactivate(){}

int Restrict2::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    int flag = 0;
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    try
    {
        if( rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection ))
            {
                set_rectangle_.x = point_.x = intersection.x;
                set_rectangle_.y = point_.y = intersection.y;
                point_.z = operation_;
                pub_.publish(point_);
                //pub2_.publish(settings_);
            }
    }
    catch (int a)
    {
        ROS_ERROR("Error Occured!!");
        return 0;
    }

    if (event.rightUp())
    {
        ROS_INFO("event.leftDown2 !!");
        operation_=true;

        set_rectangle_.z++;
        pub3_.publish(set_rectangle_);
    }
    else if( event.leftDown() )
    {
        set_rectangle_.z = 0.0;
        ROS_INFO("event.leftDown2 !!");
        pub2_.publish(settings_);
        flag |= Finished;
    }

    return flag;
}

}//end namespace

