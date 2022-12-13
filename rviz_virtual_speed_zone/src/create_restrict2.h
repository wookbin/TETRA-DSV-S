#include "ros/ros.h"
#include <rviz/tool.h>
#include <geometry_msgs/Point.h>
namespace Ogre
{
	class SceneNode;
	class Vector3;
}

namespace rviz
{
	class VectorProperty;
	class VisualizationManager;
	class ViewportMouseEvent;
	class IntProperty;
	class BoolProperty;
	class FloatProperty;
}

namespace rviz_custom
{
class Restrict2: public rviz::Tool
{
Q_OBJECT
	
public:
	Restrict2();
	virtual ~Restrict2();

	virtual void initialize();
	virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
	virtual void activate();
	virtual void deactivate();

public Q_SLOTS:
virtual void updateTopic();

private:

	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Publisher pub2_;
	ros::Publisher pub3_;
	geometry_msgs::Point point_;
	geometry_msgs::Point settings_;
	geometry_msgs::Point set_rectangle_;
	bool operation_;
	rviz::IntProperty* Size_property_2;
	rviz::BoolProperty* Remove_all_2;
	rviz::FloatProperty* float_property;
};

}//end_namespace



