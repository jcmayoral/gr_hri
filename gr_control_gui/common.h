#ifndef MYCOMMONVIZ_H
#define MYCOMMONVIZ_H

#include <vector>

#include <QWidget>
#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QListView>
#include <QList>
#include <QListWidget>

#include <QGridLayout>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QLineEdit>

#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/view_controller.h>
#include <rviz/display.h>
#include <rviz/default_plugin/marker_array_display.h>

#include <nodes_visualizer.hpp>

#include <boost/foreach.hpp>
#include <thread>

#include <gr_map_utils/UpdateMap.h>

#include <mongodb_store/message_store.h>
#include <warehouse_ros_mongo/database_connection.h>
#include <warehouse_ros_mongo/message_collection.h>


#include <visualization_msgs/MarkerArray.h>
#include <navigation_msgs/TopologicalMap.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef warehouse_ros::MessageCollection<navigation_msgs::TopologicalMap> TopoMapCollection;
typedef warehouse_ros::MessageWithMetadata<navigation_msgs::TopologicalMap> TopoMapWithMetadata;
typedef TopoMapWithMetadata::ConstPtr TopoMapMetaPtr;
namespace rviz
{
class Display;
class RenderPanel;
class ViewManager;
class ViewController;
class VisualizationManager;
}

namespace gr_control_gui{
  typedef std::map<std::string, geometry_msgs::Pose> NodeMap;
  typedef std::pair<std::string, std::string> Edges;

  class MyCommonViz: public QWidget{
    Q_OBJECT
    public:
      MyCommonViz( QWidget* parent = 0 );
      virtual ~MyCommonViz();

      void loadGUI();
      void visualizeMap();
      void publishRegion();
      virtual void updateAfterLoad() {

      };

      warehouse_ros::Metadata::Ptr makeMetadata(const TopoMapCollection& coll, const navigation_msgs::TopologicalMap& t, const std::string& name)
      {
        warehouse_ros::Metadata::Ptr meta = coll.createMetadata();
        meta->append("map_id", t.map_id);
        meta->append("frame_id", t.header.frame_id);
        meta->append("name", name);
        int nn = t.nodes.size();
        meta->append("nsize", nn);

        int ncount = 0;
        for (auto n: t.nodes){
          meta->append("node_name_" + std::to_string(ncount), n.name);
          meta->append("node_map_"+ std::to_string(ncount), n.map);
          meta->append("node_x_"+ std::to_string(ncount), n.pose.position.x);
          meta->append("node_y_"+ std::to_string(ncount), n.pose.position.y);
          meta->append("node_z_"+ std::to_string(ncount), n.pose.position.z);

          meta->append("node_ox_"+ std::to_string(ncount), n.pose.orientation.x);
          meta->append("node_oy_"+ std::to_string(ncount), n.pose.orientation.y);
          meta->append("node_oz_"+ std::to_string(ncount), n.pose.orientation.z);
          meta->append("node_ow_"+ std::to_string(ncount), n.pose.orientation.w);

          int nsize= n.verts.size();
          std::cout << "nverts " << nsize << std::endl;
          meta->append("nverts_"+ std::to_string(ncount), nsize);

          int nverts =0;
          for (auto v: n.verts){
            meta->append("node_"+ std::to_string(ncount)+"_vx_"+std::to_string(nverts), v.x);
            meta->append("node_"+ std::to_string(ncount)+"_vy_"+std::to_string(nverts), v.y);
            nverts++;
          }

          int n_edges = n.edges.size();
          int nedges = 0;
          meta->append("nedges_"+ std::to_string(ncount), n_edges);

          for (auto e: n.edges){
            meta->append("node_"+ std::to_string(ncount)+"_ex_"+std::to_string(nedges), e.edge_id);
            meta->append("node_"+ std::to_string(ncount)+"_ey_"+std::to_string(nedges), e.node);
            nedges++;
          }


          ncount++;
        }
        meta->append("info_mapframe", t.info.map_frame);
        meta->append("info_sizex", t.info.sizex);
        meta->append("info_sizey", t.info.sizey);
        meta->append("info_rrs", t.info.robot_radius);
        meta->append("info_direction", t.info.direction);
        meta->append("info_angleoffset", t.info.angle_offset);

        return meta;
      }
      //warehouse_ros_mongo
      warehouse_ros_mongo::MongoDatabaseConnection mongo_connection_;
      void parseMessage(navigation_msgs::TopologicalMap& map, TopoMapMetaPtr msg);

    protected Q_SLOTS:
      void loadMap();
    protected:
      NodeMap node_map_;
      std::string storing_id_;
      MapGenerator* map_utils_;
      QGridLayout* controls_layout_;
      QVBoxLayout* main_layout_;
      rviz::RenderPanel* render_panel_;
     	mongodb_store::MessageStoreProxy* message_store_;

      navigation_msgs::TopologicalMap load_map_;
      visualization_msgs::MarkerArray marker_array_;
      std::vector<Edges> edges_;
      std::string map_frame_;

      ros::Publisher map_publisher_;
      ros::Publisher region_publisher_;
      ros::NodeHandle nh_;
      rviz::VisualizationManager* manager_;
      ros::ServiceClient map_client_;

      //TODO Update TopoInfo
      double robot_radius_;
      float terrain_y_;
      float terrain_x_;
      int y_cells_;

      //TODO THIS ARE SAME
      int nrows_;
      int id_maxnumberrows_;
      int default_npoints_;

      //map rotation
      float angle_;
      int direction_;



  };
};
#endif
