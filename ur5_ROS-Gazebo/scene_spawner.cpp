#include <ros/ros.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

struct SceneConfig {
  double table_x;
  double table_y;
  double table_z;
  double table_size_x;
  double table_size_y;
  double table_size_z;

  double block_size_x;
  double block_size_y;
  double block_size_z;
  double block_mass;
  double block_mu;
  double block_spacing;
  double spawn_x_min;
  double spawn_x_max;
  double spawn_y_min;
  double spawn_y_max;

  double grid_origin_x;
  double grid_origin_y;
  double grid_spacing_x;
  double grid_spacing_y;
  int grid_cols;
  int grid_rows;
  double grid_cell_size;
  double grid_plate_height;

  bool randomize;
  bool spawn_obstacles;
  bool spawn_fruits;
  int seed;
};

struct Pose2D {
  double x;
  double y;
  double z;
  double yaw;
};

std::string intToString(int value) {
  std::stringstream ss;
  ss << value;
  return ss.str();
}

std::string doubleToString(double value) {
  std::stringstream ss;
  ss.precision(6);
  ss << std::fixed << value;
  return ss.str();
}

geometry_msgs::Pose makePose(double x, double y, double z, double yaw) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

std::string material(double r, double g, double b, double a) {
  std::stringstream ss;
  ss << "<material><ambient>" << r << " " << g << " " << b << " " << a
     << "</ambient><diffuse>" << r << " " << g << " " << b << " " << a
     << "</diffuse><specular>0.05 0.05 0.05 1</specular></material>";
  return ss.str();
}

std::string makeBoxVisual(const std::string& name, double x, double y, double z,
                          double sx, double sy, double sz, double r, double g,
                          double b, double a) {
  std::stringstream ss;
  ss << "<visual name='" << name << "'>"
     << "<pose>" << x << " " << y << " " << z << " 0 0 0</pose>"
     << "<geometry><box><size>" << sx << " " << sy << " " << sz
     << "</size></box></geometry>" << material(r, g, b, a) << "</visual>";
  return ss.str();
}

std::string segmentVisual(int segment_id, double x, double y, double z,
                          double length, double width, double thickness,
                          bool vertical) {
  double sx = vertical ? width : length;
  double sy = vertical ? length : width;
  std::stringstream ss;
  ss << makeBoxVisual("digit_segment_" + intToString(segment_id), x, y,
                      z + thickness * 0.5, sx, sy, thickness, 0.02, 0.02, 0.02,
                      1.0);
  return ss.str();
}

bool digitUsesSegment(int digit, int segment) {
  static const bool segments[10][7] = {
      {true, true, true, true, true, true, false},
      {false, true, true, false, false, false, false},
      {true, true, false, true, true, false, true},
      {true, true, true, true, false, false, true},
      {false, true, true, false, false, true, true},
      {true, false, true, true, false, true, true},
      {true, false, true, true, true, true, true},
      {true, true, true, false, false, false, false},
      {true, true, true, true, true, true, true},
      {true, true, true, true, false, true, true}};
  return segments[digit][segment];
}

std::string makeDigitSegments(int digit, double z, double scale) {
  const double length = scale * 0.58;
  const double side_length = scale * 0.48;
  const double width = scale * 0.10;
  const double top_y = scale * 0.29;
  const double mid_y = 0.0;
  const double bottom_y = -scale * 0.29;
  const double left_x = -scale * 0.25;
  const double right_x = scale * 0.25;

  std::stringstream ss;
  if (digitUsesSegment(digit, 0)) ss << segmentVisual(0, 0.0, top_y, z, length, width, 0.003, false);
  if (digitUsesSegment(digit, 1)) ss << segmentVisual(1, right_x, top_y * 0.5, z, side_length, width, 0.003, true);
  if (digitUsesSegment(digit, 2)) ss << segmentVisual(2, right_x, bottom_y * 0.5, z, side_length, width, 0.003, true);
  if (digitUsesSegment(digit, 3)) ss << segmentVisual(3, 0.0, bottom_y, z, length, width, 0.003, false);
  if (digitUsesSegment(digit, 4)) ss << segmentVisual(4, left_x, bottom_y * 0.5, z, side_length, width, 0.003, true);
  if (digitUsesSegment(digit, 5)) ss << segmentVisual(5, left_x, top_y * 0.5, z, side_length, width, 0.003, true);
  if (digitUsesSegment(digit, 6)) ss << segmentVisual(6, 0.0, mid_y, z, length, width, 0.003, false);
  return ss.str();
}

std::string makeTableSdf(const SceneConfig& cfg) {
  std::stringstream ss;
  ss << "<sdf version='1.6'><model name='assignment_table'><static>true</static>"
     << "<link name='link'>"
     << "<collision name='collision'><geometry><box><size>"
     << cfg.table_size_x << " " << cfg.table_size_y << " " << cfg.table_size_z
     << "</size></box></geometry><surface><friction><ode><mu>10</mu><mu2>10</mu2>"
     << "</ode></friction></surface></collision>"
     << makeBoxVisual("visual", 0, 0, 0, cfg.table_size_x, cfg.table_size_y,
                      cfg.table_size_z, 0.42, 0.42, 0.40, 1.0)
     << "</link></model></sdf>";
  return ss.str();
}

std::string makeGridCellSdf(int id, const SceneConfig& cfg) {
  std::stringstream ss;
  ss << "<sdf version='1.6'><model name='target_grid_" << id
     << "'><static>true</static><link name='link'>"
     << makeBoxVisual("plate", 0, 0, cfg.grid_plate_height * 0.5,
                      cfg.grid_cell_size, cfg.grid_cell_size,
                      cfg.grid_plate_height, 0.82, 0.82, 0.72, 1.0)
     << makeBoxVisual("border_n", 0, cfg.grid_cell_size * 0.5,
                      cfg.grid_plate_height + 0.004, cfg.grid_cell_size, 0.006,
                      0.006, 0.08, 0.20, 0.65, 1.0)
     << makeBoxVisual("border_s", 0, -cfg.grid_cell_size * 0.5,
                      cfg.grid_plate_height + 0.004, cfg.grid_cell_size, 0.006,
                      0.006, 0.08, 0.20, 0.65, 1.0)
     << makeBoxVisual("border_e", cfg.grid_cell_size * 0.5, 0,
                      cfg.grid_plate_height + 0.004, 0.006, cfg.grid_cell_size,
                      0.006, 0.08, 0.20, 0.65, 1.0)
     << makeBoxVisual("border_w", -cfg.grid_cell_size * 0.5, 0,
                      cfg.grid_plate_height + 0.004, 0.006, cfg.grid_cell_size,
                      0.006, 0.08, 0.20, 0.65, 1.0)
     << makeDigitSegments(id, cfg.grid_plate_height + 0.006,
                          cfg.grid_cell_size * 0.55)
     << "</link></model></sdf>";
  return ss.str();
}

std::string makeBlockSdf(int digit, const SceneConfig& cfg) {
  const double sx = cfg.block_size_x;
  const double sy = cfg.block_size_y;
  const double sz = cfg.block_size_z;
  const double inertia = cfg.block_mass * (sx * sx + sy * sy + sz * sz) / 12.0;

  const double color_r[10] = {0.80, 0.20, 0.18, 0.95, 0.24, 0.18, 0.78, 0.95, 0.20, 0.48};
  const double color_g[10] = {0.22, 0.48, 0.63, 0.72, 0.60, 0.70, 0.32, 0.48, 0.70, 0.32};
  const double color_b[10] = {0.18, 0.85, 0.95, 0.20, 0.28, 0.35, 0.80, 0.16, 0.40, 0.72};

  std::stringstream ss;
  ss << "<sdf version='1.6'><model name='digit_block_" << digit << "'>"
     << "<static>false</static><link name='base_link'>"
     << "<inertial><pose>0 0 " << sz * 0.5 << " 0 0 0</pose><mass>"
     << cfg.block_mass << "</mass><inertia><ixx>" << inertia
     << "</ixx><iyy>" << inertia << "</iyy><izz>" << inertia
     << "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
     << "<collision name='collision'><pose>0 0 " << sz * 0.5
     << " 0 0 0</pose><geometry><box><size>" << sx << " " << sy << " "
     << sz << "</size></box></geometry><surface><friction><ode><mu>"
     << cfg.block_mu << "</mu><mu2>" << cfg.block_mu
     << "</mu2></ode></friction></surface></collision>"
     << makeBoxVisual("body", 0, 0, sz * 0.5, sx, sy, sz, color_r[digit],
                      color_g[digit], color_b[digit], 1.0)
     << makeDigitSegments(digit, sz + 0.002, sx * 0.78)
     << "</link></model></sdf>";
  return ss.str();
}

std::string makeObstacleSdf(int id) {
  const double sx = 0.08;
  const double sy = 0.08;
  const double sz = 0.12;
  std::stringstream ss;
  ss << "<sdf version='1.6'><model name='obstacle_" << id
     << "'><static>true</static><link name='link'>"
     << "<collision name='collision'><pose>0 0 " << sz * 0.5
     << " 0 0 0</pose><geometry><box><size>" << sx << " " << sy << " "
     << sz << "</size></box></geometry></collision>"
     << makeBoxVisual("visual", 0, 0, sz * 0.5, sx, sy, sz, 0.18, 0.18, 0.18,
                      1.0)
     << "</link></model></sdf>";
  return ss.str();
}

std::string makeAppleSdf() {
  const double body_r = 0.028;
  const double mass = 0.035;
  const double inertia = 2.0 / 5.0 * mass * body_r * body_r;
  const double stem_r = 0.003;
  const double stem_h = 0.012;
  const double leaf_w = 0.015;
  const double leaf_h = 0.002;
  const double leaf_d = 0.008;
  std::stringstream ss;
  ss << "<sdf version='1.6'><model name='fruit_apple'>"
     << "<static>false</static><link name='link'>"
     << "<inertial><pose>0 0 " << body_r << " 0 0 0</pose>"
     << "<mass>" << mass << "</mass><inertia>"
     << "<ixx>" << inertia << "</ixx><iyy>" << inertia << "</iyy>"
     << "<izz>" << inertia << "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
     << "</inertia></inertial>"
     << "<collision name='body_collision'><pose>0 0 " << body_r << " 0 0 0</pose>"
     << "<geometry><sphere><radius>" << body_r << "</radius></sphere></geometry>"
     << "<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction>"
     << "<contact><ode><kp>1e6</kp><kd>100</kd></ode></contact></surface></collision>"
     << "<visual name='body_visual'><pose>0 0 " << body_r << " 0 0 0</pose>"
     << "<geometry><sphere><radius>" << body_r << "</radius></sphere></geometry>"
     << material(0.88, 0.08, 0.06, 1.0) << "</visual>"
     << "<visual name='stem_visual'><pose>0 0 " << body_r * 2 + stem_h * 0.5 << " 0 0 0</pose>"
     << "<geometry><cylinder><radius>" << stem_r << "</radius><length>" << stem_h << "</length></cylinder></geometry>"
     << material(0.35, 0.22, 0.10, 1.0) << "</visual>"
     << "<visual name='leaf_visual'><pose>" << leaf_d * 0.5 << " 0 " << body_r * 2 + stem_h * 0.3 << " 0 0 0.3</pose>"
     << "<geometry><box><size>" << leaf_d << " " << leaf_w << " " << leaf_h << "</size></box></geometry>"
     << material(0.18, 0.55, 0.12, 1.0) << "</visual>"
     << "</link></model></sdf>";
  return ss.str();
}

std::string makePearSdf() {
  const double bottom_r = 0.030;
  const double top_r = 0.018;
  const double mass = 0.04;
  const double inertia = 2.0 / 5.0 * mass * bottom_r * bottom_r;
  const double stem_r = 0.003;
  const double stem_h = 0.015;
  std::stringstream ss;
  ss << "<sdf version='1.6'><model name='fruit_pear'>"
     << "<static>false</static><link name='link'>"
     << "<inertial><pose>0 0 " << bottom_r << " 0 0 0</pose>"
     << "<mass>" << mass << "</mass><inertia>"
     << "<ixx>" << inertia << "</ixx><iyy>" << inertia << "</iyy>"
     << "<izz>" << inertia << "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
     << "</inertia></inertial>"
     << "<collision name='bottom_collision'><pose>0 0 " << bottom_r << " 0 0 0</pose>"
     << "<geometry><sphere><radius>" << bottom_r << "</radius></sphere></geometry>"
     << "<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction>"
     << "<contact><ode><kp>1e6</kp><kd>100</kd></ode></contact></surface></collision>"
     << "<collision name='top_collision'><pose>0 0 " << bottom_r * 2 - top_r * 0.3 << " 0 0 0</pose>"
     << "<geometry><sphere><radius>" << top_r << "</radius></sphere></geometry>"
     << "<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction>"
     << "<contact><ode><kp>1e6</kp><kd>100</kd></ode></contact></surface></collision>"
     << "<visual name='bottom_visual'><pose>0 0 " << bottom_r << " 0 0 0</pose>"
     << "<geometry><sphere><radius>" << bottom_r << "</radius></sphere></geometry>"
     << material(0.52, 0.75, 0.18, 1.0) << "</visual>"
     << "<visual name='top_visual'><pose>0 0 " << bottom_r * 2 - top_r * 0.3 << " 0 0 0</pose>"
     << "<geometry><sphere><radius>" << top_r << "</radius></sphere></geometry>"
     << material(0.52, 0.75, 0.18, 1.0) << "</visual>"
     << "<visual name='stem_visual'><pose>0 0 " << bottom_r * 2 + top_r * 0.5 + stem_h * 0.5 << " 0 0 0</pose>"
     << "<geometry><cylinder><radius>" << stem_r << "</radius><length>" << stem_h << "</length></cylinder></geometry>"
     << material(0.30, 0.18, 0.08, 1.0) << "</visual>"
     << "</link></model></sdf>";
  return ss.str();
}

std::string makeBananaSdf() {
  const double total_len = 0.07;
  const double base_z = 0.016;
  const double mass = 0.028;
  const double max_r = 0.015;
  const double min_r = 0.005;
  const double inertia_xx = mass * (3 * max_r * max_r + total_len * total_len) / 12.0;
  const double inertia_yy = mass * max_r * max_r / 2.0;
  const int n_seg = 20;
  const double seg_len = total_len / n_seg;
  const double tip_len = 0.008;
  std::stringstream ss;
  ss << "<sdf version='1.6'><model name='fruit_banana'>"
     << "<static>false</static><link name='link'>"
     << "<inertial><pose>0 0 " << base_z << " 0 1.5708 0</pose>"
     << "<mass>" << mass << "</mass><inertia>"
     << "<ixx>" << inertia_xx << "</ixx><iyy>" << inertia_xx << "</iyy>"
     << "<izz>" << inertia_yy << "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
     << "</inertia></inertial>"
     << "<collision name='collision'><pose>0 0 " << base_z << " 0 1.5708 0</pose>"
     << "<geometry><cylinder><radius>" << max_r << "</radius><length>" << total_len << "</length></cylinder></geometry>"
     << "<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction>"
     << "<contact><ode><kp>1e6</kp><kd>100</kd></ode></contact></surface></collision>";
  for (int i = 0; i < n_seg; ++i) {
    double x = -total_len * 0.5 + seg_len * 0.5 + i * seg_len;
    double t = static_cast<double>(i) / (n_seg - 1);
    double r = min_r + (max_r - min_r) * std::sin(t * M_PI);
    ss << "<visual name='seg_" << i << "'><pose>" << x << " 0 " << base_z << " 0 1.5708 0</pose>"
       << "<geometry><cylinder><radius>" << r << "</radius><length>" << seg_len + 0.0005 << "</length></cylinder></geometry>"
       << material(0.95, 0.82, 0.12, 1.0) << "</visual>";
  }
  ss << "<visual name='tip1_visual'><pose>" << total_len * 0.5 + tip_len * 0.4 << " 0 " << base_z << " 0 1.5708 0</pose>"
     << "<geometry><cylinder><radius>" << 0.004 << "</radius><length>" << tip_len << "</length></cylinder></geometry>"
     << material(0.45, 0.32, 0.12, 1.0) << "</visual>"
     << "<visual name='tip2_visual'><pose>-" << total_len * 0.5 + tip_len * 0.4 << " 0 " << base_z << " 0 1.5708 0</pose>"
     << "<geometry><cylinder><radius>" << 0.004 << "</radius><length>" << tip_len << "</length></cylinder></geometry>"
     << material(0.45, 0.32, 0.12, 1.0) << "</visual>"
     << "</link></model></sdf>";
  return ss.str();
}

bool spawnSdf(ros::ServiceClient& client, const std::string& name,
              const std::string& xml, const geometry_msgs::Pose& pose) {
  gazebo_msgs::SpawnModel srv;
  srv.request.model_name = name;
  srv.request.model_xml = xml;
  srv.request.robot_namespace = name;
  srv.request.initial_pose = pose;
  srv.request.reference_frame = "world";
  if (!client.call(srv)) {
    ROS_ERROR_STREAM("Failed to call /gazebo/spawn_sdf_model for " << name);
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR_STREAM("Failed to spawn " << name << ": " << srv.response.status_message);
    return false;
  }
  ROS_INFO_STREAM("Spawned " << name);
  return true;
}

bool containsModel(const std::vector<std::string>& names, const std::string& name) {
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == name) {
      return true;
    }
  }
  return false;
}

void deleteIfExists(ros::ServiceClient& client,
                    const std::vector<std::string>& existing_models,
                    const std::string& name) {
  if (!containsModel(existing_models, name)) {
    return;
  }
  gazebo_msgs::DeleteModel srv;
  srv.request.model_name = name;
  client.call(srv);
}

double uniform(double min_value, double max_value) {
  return min_value + (max_value - min_value) *
                         (static_cast<double>(std::rand()) / RAND_MAX);
}

bool farEnough(const Pose2D& candidate, const std::vector<Pose2D>& poses,
               double min_distance) {
  for (size_t i = 0; i < poses.size(); ++i) {
    const double dx = candidate.x - poses[i].x;
    const double dy = candidate.y - poses[i].y;
    if (std::sqrt(dx * dx + dy * dy) < min_distance) {
      return false;
    }
  }
  return true;
}

std::vector<Pose2D> generateBlockPoses(const SceneConfig& cfg) {
  std::vector<Pose2D> poses;
  const double z = cfg.table_z + cfg.table_size_z * 0.5 + 0.002;

  if (!cfg.randomize) {
    for (int i = 0; i < 10; ++i) {
      Pose2D pose;
      pose.x = cfg.spawn_x_min + (i % 5) * cfg.block_spacing;
      pose.y = cfg.spawn_y_min + (i / 5) * cfg.block_spacing;
      pose.z = z;
      pose.yaw = 0.0;
      poses.push_back(pose);
    }
    return poses;
  }

  for (int i = 0; i < 10; ++i) {
    Pose2D pose;
    bool accepted = false;
    for (int attempt = 0; attempt < 200 && !accepted; ++attempt) {
      pose.x = uniform(cfg.spawn_x_min, cfg.spawn_x_max);
      pose.y = uniform(cfg.spawn_y_min, cfg.spawn_y_max);
      pose.z = z;
      pose.yaw = uniform(-3.1415926, 3.1415926);
      accepted = farEnough(pose, poses, cfg.block_spacing);
    }
    if (!accepted) {
      pose.x = cfg.spawn_x_min + (i % 5) * cfg.block_spacing;
      pose.y = cfg.spawn_y_min + (i / 5) * cfg.block_spacing;
      pose.z = z;
      pose.yaw = 0.0;
    }
    poses.push_back(pose);
  }
  return poses;
}

Pose2D gridPoseForId(int id, const SceneConfig& cfg) {
  const int index = id - 1;
  Pose2D pose;
  pose.x = cfg.grid_origin_x + (index % cfg.grid_cols) * cfg.grid_spacing_x;
  pose.y = cfg.grid_origin_y + (index / cfg.grid_cols) * cfg.grid_spacing_y;
  pose.z = cfg.table_z + cfg.table_size_z * 0.5 + 0.001;
  pose.yaw = 0.0;
  return pose;
}

void loadConfig(ros::NodeHandle& pnh, SceneConfig* cfg) {
  pnh.param("table_x", cfg->table_x, 0.25);
  pnh.param("table_y", cfg->table_y, 0.0);
  pnh.param("table_z", cfg->table_z, 0.10);
  pnh.param("table_size_x", cfg->table_size_x, 1.20);
  pnh.param("table_size_y", cfg->table_size_y, 0.90);
  pnh.param("table_size_z", cfg->table_size_z, 0.20);

  pnh.param("block_size_x", cfg->block_size_x, 0.06);
  pnh.param("block_size_y", cfg->block_size_y, 0.06);
  pnh.param("block_size_z", cfg->block_size_z, 0.04);
  pnh.param("block_mass", cfg->block_mass, 0.05);
  pnh.param("block_mu", cfg->block_mu, 4.0);
  pnh.param("block_spacing", cfg->block_spacing, 0.09);
  pnh.param("spawn_x_min", cfg->spawn_x_min, 0.34);
  pnh.param("spawn_x_max", cfg->spawn_x_max, 0.75);
  pnh.param("spawn_y_min", cfg->spawn_y_min, -0.30);
  pnh.param("spawn_y_max", cfg->spawn_y_max, 0.28);

  pnh.param("grid_origin_x", cfg->grid_origin_x, -0.25);
  pnh.param("grid_origin_y", cfg->grid_origin_y, -0.22);
  pnh.param("grid_spacing_x", cfg->grid_spacing_x, 0.18);
  pnh.param("grid_spacing_y", cfg->grid_spacing_y, 0.18);
  pnh.param("grid_cols", cfg->grid_cols, 3);
  pnh.param("grid_rows", cfg->grid_rows, 2);
  pnh.param("grid_cell_size", cfg->grid_cell_size, 0.13);
  pnh.param("grid_plate_height", cfg->grid_plate_height, 0.004);

  pnh.param("randomize", cfg->randomize, true);
  pnh.param("spawn_obstacles", cfg->spawn_obstacles, false);
  pnh.param("spawn_fruits", cfg->spawn_fruits, false);
  pnh.param("seed", cfg->seed, 0);
}

std::string buildBlockTruthMessage(const std::vector<Pose2D>& block_poses) {
  std::stringstream ss;
  ss << "blocks digit x y z yaw\n";
  for (int i = 0; i < 10; ++i) {
    ss << i << " " << doubleToString(block_poses[i].x) << " "
       << doubleToString(block_poses[i].y) << " "
       << doubleToString(block_poses[i].z) << " "
       << doubleToString(block_poses[i].yaw) << "\n";
  }
  return ss.str();
}

std::string buildGridTruthMessage(const SceneConfig& cfg) {
  std::stringstream ss;
  ss << "targets id x y z yaw\n";
  for (int id = 1; id <= 6; ++id) {
    const Pose2D pose = gridPoseForId(id, cfg);
    ss << id << " " << doubleToString(pose.x) << " " << doubleToString(pose.y)
       << " " << doubleToString(pose.z) << " " << doubleToString(pose.yaw)
       << "\n";
  }
  return ss.str();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scene_spawner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  SceneConfig cfg;
  loadConfig(pnh, &cfg);
  const int seed = cfg.seed == 0 ? static_cast<int>(std::time(NULL)) : cfg.seed;
  std::srand(seed);
  ROS_INFO_STREAM("Assignment scene seed: " << seed);

  ros::service::waitForService("/gazebo/spawn_sdf_model");
  ros::service::waitForService("/gazebo/delete_model");
  ros::service::waitForService("/gazebo/get_world_properties");
  ros::ServiceClient spawn_client =
      nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  ros::ServiceClient delete_client =
      nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  ros::ServiceClient world_client =
      nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

  gazebo_msgs::GetWorldProperties world_srv;
  std::vector<std::string> existing_models;
  if (world_client.call(world_srv)) {
    existing_models = world_srv.response.model_names;
  }

  deleteIfExists(delete_client, existing_models, "assignment_table");
  for (int i = 1; i <= 6; ++i) deleteIfExists(delete_client, existing_models, "target_grid_" + intToString(i));
  for (int i = 0; i < 10; ++i) deleteIfExists(delete_client, existing_models, "digit_block_" + intToString(i));
  for (int i = 1; i <= 2; ++i) deleteIfExists(delete_client, existing_models, "obstacle_" + intToString(i));
  deleteIfExists(delete_client, existing_models, "fruit_apple");
  deleteIfExists(delete_client, existing_models, "fruit_pear");
  deleteIfExists(delete_client, existing_models, "fruit_banana");

  spawnSdf(spawn_client, "assignment_table", makeTableSdf(cfg),
           makePose(cfg.table_x, cfg.table_y, cfg.table_z, 0.0));

  for (int id = 1; id <= 6; ++id) {
    const Pose2D grid_pose = gridPoseForId(id, cfg);
    spawnSdf(spawn_client, "target_grid_" + intToString(id), makeGridCellSdf(id, cfg),
             makePose(grid_pose.x, grid_pose.y, grid_pose.z, grid_pose.yaw));

    std::vector<double> param_pose;
    param_pose.push_back(grid_pose.x);
    param_pose.push_back(grid_pose.y);
    param_pose.push_back(grid_pose.z);
    param_pose.push_back(grid_pose.yaw);
    nh.setParam("/scene/target_" + intToString(id) + "_pose", param_pose);
  }

  std::vector<Pose2D> block_poses = generateBlockPoses(cfg);
  for (int digit = 0; digit < 10; ++digit) {
    const Pose2D pose = block_poses[digit];
    spawnSdf(spawn_client, "digit_block_" + intToString(digit), makeBlockSdf(digit, cfg),
             makePose(pose.x, pose.y, pose.z, pose.yaw));

    std::vector<double> param_pose;
    param_pose.push_back(pose.x);
    param_pose.push_back(pose.y);
    param_pose.push_back(pose.z);
    param_pose.push_back(pose.yaw);
    nh.setParam("/scene/block_" + intToString(digit) + "_pose", param_pose);
  }

  if (cfg.spawn_obstacles) {
    spawnSdf(spawn_client, "obstacle_1", makeObstacleSdf(1),
             makePose(0.14, 0.09, cfg.table_z + cfg.table_size_z * 0.5, 0.0));
    spawnSdf(spawn_client, "obstacle_2", makeObstacleSdf(2),
             makePose(0.18, -0.12, cfg.table_z + cfg.table_size_z * 0.5, 0.0));
  }

  if (cfg.spawn_fruits) {
    const double fruit_z = cfg.table_z + cfg.table_size_z * 0.5 + 0.002;
    spawnSdf(spawn_client, "fruit_apple", makeAppleSdf(),
             makePose(0.80, -0.35, fruit_z, 0.0));
    spawnSdf(spawn_client, "fruit_pear", makePearSdf(),
             makePose(0.80, 0.30, fruit_z, 0.785));
    spawnSdf(spawn_client, "fruit_banana", makeBananaSdf(),
             makePose(0.78, -0.05, fruit_z, 1.5708));
    ROS_INFO("Spawned fruits: apple, pear, banana");
  }

  std_msgs::String block_truth_msg;
  block_truth_msg.data = buildBlockTruthMessage(block_poses);
  std_msgs::String grid_truth_msg;
  grid_truth_msg.data = buildGridTruthMessage(cfg);
  ros::Publisher truth_pub = nh.advertise<std_msgs::String>("/scene/block_truth", 1, true);
  ros::Publisher grid_pub = nh.advertise<std_msgs::String>("/scene/grid_truth", 1, true);

  ROS_INFO_STREAM("Assignment scene ready:\n" << block_truth_msg.data << grid_truth_msg.data);
  ros::Rate rate(1.0);
  while (ros::ok()) {
    truth_pub.publish(block_truth_msg);
    grid_pub.publish(grid_truth_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
