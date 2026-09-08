#include "octomap_rviz_plugins/occupancy_grid_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <QObject>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <sstream>

#include "mapping_module/world_representation/ig_tree.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/visualization_manager.h"

using namespace rviz;

namespace octomap_rviz_plugin {

// backward::SignalHandling OccupancyGridDisplay::sh_ =
// backward::SignalHandling();

static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

enum OctreeVoxelRenderMode {
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2,
  OCTOMAP_ALL_VOXELS = 3,
  OCTOMAP_NO_MEASUREMENT_VOXELS = 4,
  OCTOMAP_FRONTIERS = 5,
  OCTOMAP_TERRAIN = 6
};

enum OctreeVoxelColorMode {
  OCTOMAP_CELL_COLOR,
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
  OCTOMAP_TSDF_DISTANCE_COLOR,
  OCTOMAP_TSDF_WEIGHT_COLOR,
  OCTOMAP_ROUGHNESS_COLOR,
  OCTOMAP_SLOPE_COLOR,
  OCTOMAP_SPARSITY_COLOR,
  OCTOMAP_HEIGHT_DIFF_COLOR,
  OCTOMAP_TRAVERSABLE_COLOR,
  OCTOMAP_COLLISION_COLOR,
  OCTOMAP_CONNECTED_COLOR
};

OccupancyGridDisplay::OccupancyGridDisplay()
    : rviz::Display(),
      new_points_received_(false),
      messages_received_(0),
      queue_size_(5),
      color_factor_(0.8) {
  static octomap::IgTree ig_tree(0.1);
  octomap_topic_property_ = new RosTopicProperty(
      "Octomap Topic", "",
      QString::fromStdString(
          ros::message_traits::datatype<octomap_msgs::Octomap>()),
      "octomap_msgs::Octomap topic to subscribe to (binary or full probability "
      "map)",
      this, SLOT(updateTopic()));

  queue_size_property_ = new IntProperty(
      "Queue Size", queue_size_,
      "Advanced: set the size of the incoming message queue.  Increasing this "
      "is useful if your incoming TF data is delayed significantly from your"
      " image data, but it can greatly increase memory usage if the messages "
      "are big.",
      this, SLOT(updateQueueSize()));
  queue_size_property_->setMin(1);

  octree_render_property_ = new rviz::EnumProperty(
      "Voxel Rendering", "Occupied Voxels", "Select voxel type.", this,
      SLOT(updateOctreeRenderMode()));

  octree_render_property_->addOption("Occupied Voxels",
                                     OCTOMAP_OCCUPIED_VOXELS);
  octree_render_property_->addOption("Free Voxels", OCTOMAP_FREE_VOXELS);
  octree_render_property_->addOption("No Measurement Voxels",
                                     OCTOMAP_NO_MEASUREMENT_VOXELS);
  octree_render_property_->addOption(
      "All Voxels", OCTOMAP_FREE_VOXELS | OCTOMAP_OCCUPIED_VOXELS);
  octree_render_property_->addOption("Frontiers", OCTOMAP_FRONTIERS);
  octree_render_property_->addOption("Terrain", OCTOMAP_TERRAIN);

  octree_coloring_property_ = new rviz::EnumProperty(
      "Voxel Coloring", "Z-Axis", "Select voxel coloring mode", this,
      SLOT(updateOctreeColorMode()));

  octree_coloring_property_->addOption("Cell Color", OCTOMAP_CELL_COLOR);
  octree_coloring_property_->addOption("Z-Axis", OCTOMAP_Z_AXIS_COLOR);
  octree_coloring_property_->addOption("Cell Probability",
                                       OCTOMAP_PROBABLILTY_COLOR);
  octree_coloring_property_->addOption("TSDF Distance",
                                       OCTOMAP_TSDF_DISTANCE_COLOR);
  octree_coloring_property_->addOption("TSDF Weight",
                                       OCTOMAP_TSDF_WEIGHT_COLOR);

  octree_coloring_property_->addOption("Roughness", OCTOMAP_ROUGHNESS_COLOR);
  octree_coloring_property_->addOption("Slope", OCTOMAP_SLOPE_COLOR);
  octree_coloring_property_->addOption("Sparsity", OCTOMAP_SPARSITY_COLOR);
  octree_coloring_property_->addOption("Height Diff",
                                       OCTOMAP_HEIGHT_DIFF_COLOR);
  octree_coloring_property_->addOption("Traversability",
                                       OCTOMAP_TRAVERSABLE_COLOR);
  octree_coloring_property_->addOption("Collision", OCTOMAP_COLLISION_COLOR);
  octree_coloring_property_->addOption("Connected", OCTOMAP_CONNECTED_COLOR);
  alpha_property_ = new rviz::FloatProperty("Voxel Alpha", 1.0,
                                            "Set voxel transparency alpha",
                                            this, SLOT(updateAlpha()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  tree_depth_property_ = new IntProperty("Max. Octree Depth", max_octree_depth_,
                                         "Defines the maximum tree depth", this,
                                         SLOT(updateTreeDepth()));
  tree_depth_property_->setMin(0);

  max_height_property_ = new FloatProperty(
      "Max. Height Display", std::numeric_limits<double>::infinity(),
      "Defines the maximum height to display", this, SLOT(updateMaxHeight()));

  min_height_property_ = new FloatProperty(
      "Min. Height Display", -std::numeric_limits<double>::infinity(),
      "Defines the minimum height to display", this, SLOT(updateMinHeight()));

  max_value_property_ = new FloatProperty(
      "Max. Value Color", 1, "Defines the maximum value to display", this);

  min_value_property_ = new FloatProperty(
      "Min. Value Color", 0, "Defines the minimum value to display", this);
}

void OccupancyGridDisplay::onInitialize() {
  boost::mutex::scoped_lock lock(mutex_);

  box_size_.resize(max_octree_depth_);
  cloud_.resize(max_octree_depth_);
  point_buf_.resize(max_octree_depth_);
  new_points_.resize(max_octree_depth_);

  for (std::size_t i = 0; i < max_octree_depth_; ++i) {
    std::stringstream sname;
    sname << "PointCloud Nr." << i;
    cloud_[i] = new rviz::PointCloud();
    cloud_[i]->setName(sname.str());
    cloud_[i]->setRenderMode(rviz::PointCloud::RM_BOXES);
    scene_node_->attachObject(cloud_[i]);
  }
}

OccupancyGridDisplay::~OccupancyGridDisplay() {
  std::size_t i;

  unsubscribe();

  for (std::vector<rviz::PointCloud *>::iterator it = cloud_.begin();
       it != cloud_.end(); ++it) {
    delete *(it);
  }

  if (scene_node_) scene_node_->detachAllObjects();
}

void OccupancyGridDisplay::updateQueueSize() {
  queue_size_ = queue_size_property_->getInt();

  subscribe();
}

void OccupancyGridDisplay::onEnable() {
  scene_node_->setVisible(true);
  subscribe();
}

void OccupancyGridDisplay::onDisable() {
  scene_node_->setVisible(false);
  unsubscribe();

  clear();
}

void OccupancyGridDisplay::subscribe() {
  if (!isEnabled()) {
    return;
  }

  try {
    unsubscribe();

    const std::string &topicStr = octomap_topic_property_->getStdString();

    if (!topicStr.empty()) {
      sub_.reset(new message_filters::Subscriber<octomap_msgs::Octomap>());

      sub_->subscribe(threaded_nh_, topicStr, queue_size_);
      sub_->registerCallback(boost::bind(
          &OccupancyGridDisplay::incomingMessageCallback, this, _1));
    }
  } catch (ros::Exception &e) {
    setStatus(StatusProperty::Error, "Topic",
              (std::string("Error subscribing: ") + e.what()).c_str());
  }
}

void OccupancyGridDisplay::unsubscribe() {
  clear();

  try {
    // reset filters
    sub_.reset();
  } catch (ros::Exception &e) {
    setStatus(StatusProperty::Error, "Topic",
              (std::string("Error unsubscribing: ") + e.what()).c_str());
  }
}

// method taken from octomap_server package
void OccupancyGridDisplay::setColor(double z_pos, double min_z, double max_z,
                                    double color_factor,
                                    rviz::PointCloud::Point &point) {
  int i;
  double m, n, f;

  double s = 1.0;
  double v = 1.0;

  double h =
      (1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) *
      color_factor;

  h -= floor(h);
  h *= 6;
  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      point.setColor(v, n, m);
      break;
    case 1:
      point.setColor(n, v, m);
      break;
    case 2:
      point.setColor(m, v, n);
      break;
    case 3:
      point.setColor(m, n, v);
      break;
    case 4:
      point.setColor(n, m, v);
      break;
    case 5:
      point.setColor(v, m, n);
      break;
    default:
      point.setColor(1, 0.5, 0.5);
      break;
  }
}

void OccupancyGridDisplay::updateTreeDepth() { updateTopic(); }

void OccupancyGridDisplay::updateOctreeRenderMode() { updateTopic(); }

void OccupancyGridDisplay::updateOctreeColorMode() { updateTopic(); }

void OccupancyGridDisplay::updateAlpha() { updateTopic(); }

void OccupancyGridDisplay::updateMaxHeight() { updateTopic(); }

void OccupancyGridDisplay::updateMinHeight() { updateTopic(); }

void OccupancyGridDisplay::clear() {
  boost::mutex::scoped_lock lock(mutex_);

  // reset rviz pointcloud boxes
  for (size_t i = 0; i < cloud_.size(); ++i) {
    cloud_[i]->clear();
  }
}

void OccupancyGridDisplay::update(float wall_dt, float ros_dt) {
  if (new_points_received_) {
    boost::mutex::scoped_lock lock(mutex_);

    for (size_t i = 0; i < max_octree_depth_; ++i) {
      double size = box_size_[i];

      cloud_[i]->clear();
      cloud_[i]->setDimensions(size, size, size);

      cloud_[i]->addPoints(&new_points_[i].front(), new_points_[i].size());
      new_points_[i].clear();
      cloud_[i]->setAlpha(alpha_property_->getFloat());
    }
    new_points_received_ = false;
  }
  updateFromTF();
}

void OccupancyGridDisplay::reset() {
  clear();
  messages_received_ = 0;
  setStatus(StatusProperty::Ok, "Messages",
            QString("0 binary octomap messages received"));
}

void OccupancyGridDisplay::updateTopic() {
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

template <typename OcTreeType>
bool TemplatedOccupancyGridDisplay<OcTreeType>::checkType(std::string type_id) {
  // General case: Need to be specialized for every used case
  setStatus(StatusProperty::Warn, "Messages",
            QString("Cannot verify octomap type"));
  return true;  // Try deserialization, might crash though
}

template <>
bool TemplatedOccupancyGridDisplay<octomap::OcTreeStamped>::checkType(
    std::string type_id) {
  if (type_id == "OcTreeStamped")
    return true;
  else
    return false;
}
template <>
bool TemplatedOccupancyGridDisplay<octomap::OcTree>::checkType(
    std::string type_id) {
  if (type_id == "OcTree")
    return true;
  else
    return false;
}

template <>
bool TemplatedOccupancyGridDisplay<octomap::ColorOcTree>::checkType(
    std::string type_id) {
  if (type_id == "ColorOcTree")
    return true;
  else
    return false;
}

template <>
bool TemplatedOccupancyGridDisplay<octomap::IgTree>::checkType(
    std::string type_id) {
  if (type_id == "IgTree")
    return true;
  else
    return false;
}

template <typename OcTreeType>
bool TemplatedOccupancyGridDisplay<OcTreeType>::setVoxelColor(
    PointCloud::Point &newPoint, typename OcTreeType::NodeType &node,
    double minZ, double maxZ) {
  OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(
      octree_coloring_property_->getOptionInt());
  float cell_probability;
  switch (octree_color_mode) {
    case OCTOMAP_CELL_COLOR:
      setStatus(StatusProperty::Error, "Messages",
                QString("Cannot extract color"));
      break;
    case OCTOMAP_TSDF_DISTANCE_COLOR:
      setStatus(StatusProperty::Error, "Messages",
                QString("Cannot extract TSDF distance"));
      break;
    case OCTOMAP_TSDF_WEIGHT_COLOR:
      setStatus(StatusProperty::Error, "Messages",
                QString("Cannot extract TSDF weight"));
      break;
    case OCTOMAP_Z_AXIS_COLOR:
      setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
      break;
    case OCTOMAP_PROBABLILTY_COLOR:
      cell_probability = node.getOccupancy();
      newPoint.setColor((1.0f - cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
  return true;
}

// Specialization for ColorOcTreeNode, which can set the voxel color from the
// node itself
template <>
bool TemplatedOccupancyGridDisplay<octomap::ColorOcTree>::setVoxelColor(
    PointCloud::Point &newPoint, octomap::ColorOcTree::NodeType &node,
    double minZ, double maxZ) {
  float cell_probability;
  OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(
      octree_coloring_property_->getOptionInt());
  switch (octree_color_mode) {
    case OCTOMAP_CELL_COLOR: {
      const float b2f = 1. / 256.;
      octomap::ColorOcTreeNode::Color &color = node.getColor();
      newPoint.setColor(b2f * color.r, b2f * color.g, b2f * color.b,
                        node.getOccupancy());
      break;
    }
    case OCTOMAP_TSDF_DISTANCE_COLOR:
      setStatus(StatusProperty::Error, "Messages",
                QString("Cannot extract TSDF distance"));
      break;
    case OCTOMAP_TSDF_WEIGHT_COLOR:
      setStatus(StatusProperty::Error, "Messages",
                QString("Cannot extract TSDF weight"));
      break;
    case OCTOMAP_Z_AXIS_COLOR:
      setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
      break;
    case OCTOMAP_PROBABLILTY_COLOR:
      cell_probability = node.getOccupancy();
      newPoint.setColor((1.0f - cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
  return true;
}

// Specialization for IgOcTreeNode, which can set the voxel color from the
// node itself
template <>
bool TemplatedOccupancyGridDisplay<octomap::IgTree>::setVoxelColor(
    PointCloud::Point &newPoint, octomap::IgTree::NodeType &node, double minZ,
    double maxZ) {
  float cell_probability;
  OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(
      octree_coloring_property_->getOptionInt());
  int render_mode_mask = octree_render_property_->getOptionInt();
  double maxValue = max_value_property_->getFloat();
  double minValue = min_value_property_->getFloat();

  switch (octree_color_mode) {
    case OCTOMAP_CELL_COLOR: {
      const float b2f = 1. / 256.;
      octomap::IgTreeNode::Color &color = node.getColor();
      // if (node.hasMeasurement()) {
      // } else {
      //   newPoint.setColor(0, 0, 0, node.getOccupancy());
      // }
      newPoint.setColor(b2f * color.r, b2f * color.g, b2f * color.b,
                        node.getOccupancy());
      break;
    }
    case OCTOMAP_TSDF_DISTANCE_COLOR: {
      double tsdf_dist = node.getTsdf_Distance();
      setColor(tsdf_dist, minValue, maxValue, color_factor_, newPoint);
      break;
    }
    case OCTOMAP_TSDF_WEIGHT_COLOR: {
      double tsdf_weight = node.getTsdf_Weight();
      setColor(tsdf_weight, minValue, maxValue, color_factor_, newPoint);
      break;
    }
    case OCTOMAP_Z_AXIS_COLOR: {
      setColor(newPoint.position.z, minValue, maxValue, color_factor_, newPoint);
      break;
    }
    case OCTOMAP_PROBABLILTY_COLOR: {
      cell_probability = node.getOccupancy();
      setColor(cell_probability, minValue, maxValue, color_factor_, newPoint);
      break;
    }
    case OCTOMAP_ROUGHNESS_COLOR: {
      if (render_mode_mask == OCTOMAP_TERRAIN) {
        float c = node.getRoughness();
        setColor(c, minValue, maxValue, color_factor_, newPoint);
      }
      break;
    }
    case OCTOMAP_SLOPE_COLOR: {
      if (render_mode_mask == OCTOMAP_TERRAIN) {
        float c = node.getSlope();
        setColor(c, minValue, maxValue, color_factor_, newPoint);
      }
      break;
    }
    case OCTOMAP_SPARSITY_COLOR: {
      if (render_mode_mask == OCTOMAP_TERRAIN) {
        float c = node.getSparsity();
        setColor(c, minValue, maxValue, color_factor_, newPoint);
      }
      break;
    }
    case OCTOMAP_HEIGHT_DIFF_COLOR: {
      if (render_mode_mask == OCTOMAP_TERRAIN) {
        float c = node.getHeightDiff();
        setColor(c, minValue, maxValue, color_factor_, newPoint);
      }
      break;
    }
    case OCTOMAP_TRAVERSABLE_COLOR: {
      if (render_mode_mask == OCTOMAP_TERRAIN) {
        float c = node.getTraversability();
        setColor(c, minValue, maxValue, color_factor_, newPoint);
      }
      break;
    }
    case OCTOMAP_COLLISION_COLOR: {
      if (render_mode_mask == OCTOMAP_TERRAIN) {
        float c = node.getCollision() ? 1 : 0.5;
        setColor(c, minValue, maxValue, color_factor_, newPoint);
      }
      break;
    }
    case OCTOMAP_CONNECTED_COLOR: {
      if (render_mode_mask == OCTOMAP_TERRAIN) {
        float c = node.getConnected() ? 0.5 : 1;
        setColor(c, minValue, maxValue, color_factor_, newPoint);
      }
      break;
    }    
    default:
      break;
  }
  return true;
}

bool OccupancyGridDisplay::updateFromTF() {
  // get tf transform
  Ogre::Vector3 pos;
  Ogre::Quaternion orient;
  if (!context_->getFrameManager()->getTransform(header_, pos, orient)) {
    return false;
  }

  scene_node_->setOrientation(orient);
  scene_node_->setPosition(pos);
  return true;
}

template <typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::incomingMessageCallback(
    const octomap_msgs::OctomapConstPtr &msg) {
  ++messages_received_;
  setStatus(StatusProperty::Ok, "Messages",
            QString::number(messages_received_) + " octomap messages received");
  setStatusStd(StatusProperty::Ok, "Type", msg->id.c_str());
  if (!checkType(msg->id)) {
    setStatusStd(StatusProperty::Error, "Message",
                 "1Wrong octomap type. Use a different display type.");
    return;
  }

  ROS_DEBUG("Received OctomapBinary message (size: %d bytes)",
            (int)msg->data.size());

  header_ = msg->header;
  if (!updateFromTF()) {
    std::stringstream ss;
    ss << "Failed to transform from frame [" << header_.frame_id
       << "] to frame [" << context_->getFrameManager()->getFixedFrame() << "]";
    setStatusStd(StatusProperty::Error, "Message", ss.str());
    return;
  }

  OcTreeType *octomap = NULL;
  octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octomap = dynamic_cast<OcTreeType *>(tree);
    if (!octomap) {
      setStatusStd(StatusProperty::Error, "Message",
                   "Wrong octomap type. Use a different display type.");
    }
  } else {
    setStatusStd(StatusProperty::Error, "Message",
                 "Failed to deserialize octree message.");
    return;
  }

  tree_depth_property_->setMax(octomap->getTreeDepth());

  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);

  // reset rviz pointcloud classes
  for (std::size_t i = 0; i < max_octree_depth_; ++i) {
    point_buf_[i].clear();
    box_size_[i] = octomap->getNodeSize(i + 1);
  }

  size_t pointCount = 0;
  {
    // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(
        tree_depth_property_->getInt(), octomap->getTreeDepth());
    double maxHeight = std::min<double>(max_height_property_->getFloat(), maxZ);
    double minHeight = std::max<double>(min_height_property_->getFloat(), minZ);

    // pruning occluded voxels
    int stepSize = 1 << (octomap->getTreeDepth() - treeDepth);
    for (typename OcTreeType::iterator it = octomap->begin(treeDepth),
                                       end = octomap->end();
         it != end; ++it) {
      if (it.getZ() <= maxHeight && it.getZ() >= minHeight) {
        int render_mode_mask = octree_render_property_->getOptionInt();
        bool display_voxel = true;

        if (octomap->getTreeType() == "IgTree") {
          octomap::IgTreeNode *ig_node =
              reinterpret_cast<octomap::IgTreeNode *>(&(*it));

          // NO_MEAS && hasMeas, !NO_MEAS && !hasMeas
          if ((!ig_node->hasMeasurement()) ^
              (render_mode_mask == OCTOMAP_NO_MEASUREMENT_VOXELS))
            continue;

          if ((render_mode_mask == OCTOMAP_FREE_VOXELS &&
               octomap->isNodeOccupied(*it)) ||
              (render_mode_mask == OCTOMAP_OCCUPIED_VOXELS &&
               !octomap->isNodeOccupied(*it)))
            continue;

          if (render_mode_mask ==
              OCTOMAP_ALL_VOXELS)  // show occupied and frontiers
            // skip free && non-frontier voxels
            if (!octomap->isNodeOccupied(ig_node) && !ig_node->isFrontier())
              continue;

          if (render_mode_mask == OCTOMAP_FRONTIERS)
            // skip occupied && non-frontier voxels
            if (octomap->isNodeOccupied(ig_node) || !ig_node->isFrontier())
              continue;

          if (render_mode_mask == OCTOMAP_TERRAIN)
            if (!octomap->isNodeOccupied(ig_node)) continue;
        } else {
          if ((render_mode_mask == OCTOMAP_FREE_VOXELS &&
               octomap->isNodeOccupied(*it)) ||
              (render_mode_mask == OCTOMAP_OCCUPIED_VOXELS &&
               !octomap->isNodeOccupied(*it)))
            continue;
        }

        PointCloud::Point newPoint;
        newPoint.position.x = it.getX();
        newPoint.position.y = it.getY();
        newPoint.position.z = it.getZ();

        if (!setVoxelColor(newPoint, *it, minZ, maxZ)) continue;

        // push to point vectors
        unsigned int depth = it.getDepth();
        point_buf_[depth - 1].push_back(newPoint);

        ++pointCount;
      }
    }

    if (pointCount) {
      boost::mutex::scoped_lock lock(mutex_);

      new_points_received_ = true;

      for (size_t i = 0; i < max_octree_depth_; ++i)
        new_points_[i].swap(point_buf_[i]);
    }
    delete octomap;
  }
}
}  // namespace octomap_rviz_plugin
#include <pluginlib/class_list_macros.h>

typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::OcTree>
    OcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::ColorOcTree>
    ColorOcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::IgTree>
    IgOcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<
    octomap::OcTreeStamped>
    OcTreeStampedGridDisplay;

PLUGINLIB_EXPORT_CLASS(OcTreeGridDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(ColorOcTreeGridDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(IgOcTreeGridDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(OcTreeStampedGridDisplay, rviz::Display)
