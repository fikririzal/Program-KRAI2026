/**
 * seq_node.cpp  —  sekuensial bergaya mikrokontroler
 *
 * Alur:
 *   tunggu START (gui_start_)
 *     → seqSpearhead()  (ambil spearhead, koreksi TOF, mundur, putar)
 *   tunggu START lagi
 *     → seqForest()     (tulis sendiri)
 *   ...dst
 */

#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

#include "eira_msgs/msg/forest_plan_step.hpp"
#include "eira_msgs/msg/gui_data.hpp"
#include "eira_msgs/msg/kfs_retrieval_plan.hpp"
#include "eira_msgs/msg/web_gui_data.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware1_com/msg/lifter_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_tof/msg/tof_array.hpp"
#include "sensor_tof/msg/tof_gripper.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using ForestPlanStep = eira_msgs::msg::ForestPlanStep;

// ═════════════════════════════════════════════════════════════════════════════
//  TUNING PARAMS  — edit di sini untuk semua fungsi kontrol
// ═════════════════════════════════════════════════════════════════════════════

// ── spearhead TOF posisi ──────────────────────────────────────────────────────
static constexpr float kSpearBiru[7]  = {0.f, 330.f, 530.f, 730.f, 930.f, 0.f, 0.f};
static constexpr float kSpearMerah[7] = {0.f,  90.f, 290.f, 500.f, 0.f, 0.f, 0.f};
static constexpr int16_t kDyn1Ambil   = 2944;
static constexpr int16_t kDyn1Atas    = 1914;

// ── spearhead: maju ke posisi (TOF depan + lateral) ──────────────────────────
static constexpr float kSpearKpVx   = 0.001f;   // kp TOF depan → vx
static constexpr float kSpearMaxVx  = 0.25f;    // batas vx maju
static constexpr float kSpearTolVx  = 20.f;     // toleransi depan (mm)
static constexpr float kSpearKpVy   = 0.0015f;  // kp TOF lateral → vy
static constexpr float kSpearMaxVy  = 0.10f;    // batas vy
static constexpr float kSpearTolVy  = 5.f;      // toleransi lateral (mm)
static constexpr float kSpearSlowVy = 50.f;     // err < ini → vy lambat (0.05)
static constexpr float kSpearKpWz   = 0.02f;    // kp IMU yaw → wz (spearhead)
static constexpr float kSpearMaxWz  = 0.10f;    // batas wz

// ── spearhead: mundur ke 1000 mm ─────────────────────────────────────────────
static constexpr float kRevTarget   = 1000.f;
static constexpr float kRevKp       = 0.0008f;
static constexpr float kRevMaxVx    = 0.45f;
static constexpr float kRevTol      = 20.f;

// ── spearhead: putar 90° IMU ──────────────────────────────────────────────────
static constexpr float kSpearRotKpWz  = 0.02f;
static constexpr float kSpearRotMaxWz = 0.30f;
static constexpr float kSpearRotTol   = 2.f;    // toleransi yaw (deg)

// ── approach forest: menuju posisi KFS ───────────────────────────────────────
//   vx: rata-rata (tof_s1+tof_s2)/2, target kApproachTargetFront
//   vy: wall_kanan_ (biru) / wall_kiri_ (merah) — target per F1/F2/F3
//   wz: selisih tof_s1 - tof_s2 → paralel dinding depan
static constexpr float kApproachTargetFront = 300.f;   // target TOF depan (mm)
static constexpr float kApproachKpVx        = 0.001f;
static constexpr float kApproachMaxVx       = 0.20f;
static constexpr float kApproachTolVx       = 20.f;
static constexpr float kApproachKpVy        = 0.0015f; // wall err dalam mm-skala
static constexpr float kApproachMaxVy       = 0.30f;
static constexpr float kApproachTolVy       = 20.f;
static constexpr float kApproachSlowVy      = 50.f;    // err < ini → vy lambat
static constexpr float kApproachKpWz        = 0.001f;  // dari selisih tof (mm)
static constexpr float kApproachMaxWz       = 0.10f;

// ── scan lidar control (seqForestPos2 + scan kanan) ──────────────────────────
static constexpr float kScanKpVy   = 0.0015f;  // kp scan lidar → vy
static constexpr float kScanMaxVy  = 0.80f;    // batas vy scan
static constexpr float kScanTolVy  = 30.f;     // toleransi scan (mm)

// ── place box (stepPlaceBox) ──────────────────────────────────────────────────
static constexpr float kPlaceMaxVx = 0.30f;    // batas vx (avg s1,s2 → 350mm)
static constexpr float kPlaceMaxVy = 0.30f;    // batas vy (scan kiri/kanan → target rak)
// rak 3 (pos 6,9): pakai scan KANAN (idx 0), target berbeda karena ukur dari sisi berlawanan
static constexpr float kPlaceRak3KananG12 = 800.f;  // scan kanan target gripper 1,2
static constexpr float kPlaceRak3KananG34 = 400.f;  // scan kanan target gripper 3,4

// wall target (mm) per index: 0=unused, 1=F1, 2=F2, 3=F3
static constexpr float kForestWallBiru[8]  = {0.f, 4090.f, 2900.f, 1800.f, 0.f, 0.f, 0.f, 0.f};
static constexpr float kForestWallMerah[8] = {0.f, 4090.f, 2900.f, 1800.f, 0.f, 0.f, 0.f, 0.f};

// ── rotate (ACTION_ROTATE) ────────────────────────────────────────────────────
static constexpr float kRotateKpWz  = 0.02f;
static constexpr float kRotateMaxWz = 0.35f;
static constexpr float kRotateTol   = 2.f;

// ── move (ACTION_MOVE): gerak sambil koreksi heading ─────────────────────────
static constexpr float kMoveSpeed      = 0.22f;
static constexpr float kMoveDownSpeed = 0.17f; // khusus MOVE DOWN dan EXIT DOWN
static constexpr float kMoveKpWz    = 0.015f;
static constexpr float kMoveMaxWz   = 0.20f;
static constexpr int   kMoveMaxMs   = 3000;    // timeout per step (ms)

// ── elev delta → posisi lifter body ──────────────────────────────────────────
static constexpr int16_t kElevDelta200 = -500;   // delta=200mm → lifterHw(-500)
static constexpr int16_t kElevDelta400 = -1000;  // delta=400mm → lifterHw(-1000)

// ── lifter body saat APPROACH per target forest ───────────────────────────────
static constexpr int16_t kApproachLifter[4] = {0, -1000, -500, -1000}; // [0]=unused, [1]=F1, [2]=F2, [3]=F3

// ── pickup KFS ────────────────────────────────────────────────────────────────
static constexpr int16_t kGriperOpen        =    1;  // buka gripper
static constexpr int16_t kGriperClose       =    0;  // tutup gripper
static constexpr int16_t kPickupLifterReady = -1000; // lifter turet posisi siap
static constexpr int16_t kPickupLifterDown  = -3000; // lifter turet turun penuh ke KFS
static constexpr int16_t kPickupSudutPre    =   600; // sudut turet sebelum turun
static constexpr int16_t kPickupSudutGrab   =   740; // sudut turet saat ambil KFS
static constexpr int     kPickupWaitMs      =  1000; // tunggu gripper menutup (ms)

// ── alias untuk kompatibilitas dengan seqSpearhead() ─────────────────────────
static constexpr float kTofKp    = kSpearKpVx;
static constexpr float kTofMax   = kSpearMaxVx;
static constexpr float kTolFwd   = kSpearTolVx;
static constexpr float kWallKp   = kSpearKpVy;
static constexpr float kWallTol  = kSpearTolVy;
static constexpr float kWallSlow = kSpearSlowVy;
static constexpr float kPosKpYaw = kSpearKpWz;
static constexpr float kPosMaxWz = kSpearMaxWz;

class SeqNode : public rclcpp::Node
{
public:
  SeqNode() : Node("seq_node")
  {
    // ── subscriber GUI ────────────────────────────────────────────────────
    step_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/gui/step", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) { gui_step_ = msg->data; });

    gui_sub_ = create_subscription<eira_msgs::msg::GuiData>(
      "/gui_data", 10,
      [this](const eira_msgs::msg::GuiData::SharedPtr msg) {
        gui_team_      = msg->team; // blue = 0, red = 1
        gui_spearhead_ = msg->spearhead; // int 1-6
        gui_kfsret_    = msg->kfsret; // int 0 = 2, 1=3, 2=4
        gui_ready_     = msg->ready; // bool false / true
        gui_start_     = msg->start; // bool false / true
        gui_forest_    = msg->forest;
        gui_tictactoe_ = msg->tictactoe; // array value 4 = 9 (di eksekusi urutannya mulai dari array 0)
        gui_retry_     = msg->retry; // array 0 = spearhead, 1 = assembly, 2 = forest, 3 = arena

        if (msg->retry.size() > 5) {
          bool gui_estop = (msg->retry[5] != 0);
          if (gui_estop && !e_stop_) {
            e_stop_ = true;
            stop();
            for (int i = 0; i < 4; i++) gripper_used_[i] = false;
            RCLCPP_WARN(get_logger(), "=== E-STOP aktif dari GUI retry[5] ===");
          } else if (!gui_estop && e_stop_) {
            e_stop_ = false;
            RCLCPP_WARN(get_logger(), "=== E-STOP dilepas dari GUI retry[5] ===");
          }
        }
      
      });

    plan_sub_ = create_subscription<eira_msgs::msg::KfsRetrievalPlan>(
      "/intelligent/kfs_retrieval_plan", 10,
      [this](const eira_msgs::msg::KfsRetrievalPlan::SharedPtr msg) {
        // hanya reset jika stamp berbeda (plan baru)
        if (msg->stamp.sec == plan_stamp_sec_ && msg->stamp.nanosec == plan_stamp_nanosec_) return;
        plan_stamp_sec_     = msg->stamp.sec;
        plan_stamp_nanosec_ = msg->stamp.nanosec;
        plan_steps_         = msg->steps;
        plan_step_count_    = msg->step_count;
        plan_step_idx_      = 0;
        plan_path_nodes_    = msg->path_nodes;
        plan_picked_kfs_    = msg->picked_kfs;
        plan_total_cost_    = msg->total_cost;

      });

    // ── subscriber TOF gripper ────────────────────────────────────────────
    tof_gripper_sub_ = create_subscription<sensor_tof::msg::TofGripper>(
      "tof/gripper", 10,
      [this](const sensor_tof::msg::TofGripper::SharedPtr msg) {
        for (int i = 0; i < 8; ++i) tof_gripper_[i] = msg->dist[i];
      });

    // ── subscriber sensor ─────────────────────────────────────────────────
    tof_sub_ = create_subscription<sensor_tof::msg::TofArray>(
      "tof/array", 10,
      [this](const sensor_tof::msg::TofArray::SharedPtr msg) {
        tof_s1_      = msg->sensor1_depan;
        tof_s2_      = msg->sensor2_depan;
        tof_s3_      = msg->sensor1_kiri;
        tof_s4_      = msg->sensor2_kanan;
        tof_mid1_    = msg->sensor_mid1;
        tof_mid1_2_  = msg->sensor_mid1_2;
        tof_mid2_    = msg->sensor_mid2;
        tof_mid3_    = msg->sensor_mid3;
        tof_mid4_    = msg->sensor_mid4;
      });

    wall_kanan_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/wall/kanan", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { wall_kanan_ = msg->data; });

    wall_kiri_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/wall/kiri", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { wall_kiri_ = msg->data; });

    imu_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/imu_yaw", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        float y = msg->data;
        while (y >  180.f) y -= 360.f;
        while (y < -180.f) y += 360.f;
        imu_yaw_ = y;
      });

    cam_sub_ = create_subscription<std_msgs::msg::String>(
      "/servo/target_xy", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        const std::string & s = msg->data;
        auto px = s.find("x="), pd = s.find("depth="), py = s.find("y=");
        if (px == std::string::npos || pd == std::string::npos) {
          cam_has_data_ = false; return;
        }
        try {
          cam_x_     = std::stoi(s.substr(px + 2));
          cam_depth_ = std::stof(s.substr(pd + 6));
          cam_y_     = (py != std::string::npos) ? std::stoi(s.substr(py + 2)) : 0;
          cam_has_data_ = true;
        } catch (...) { cam_has_data_ = false; }
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = *msg;
        // ── Hokuyo UST-20LX · 270° · 1081 titik · 0.25°/titik · 0.06–20m · 25Hz ──
        //
        //                  [540] DEPAN
        //                     ^
        //          [900]      |      [180]
        //     kiri <-  .......|..../=]
        // /.. -> kanan

        //   [1080]\   [810]  [*]  [270]  /[0]
        //          \                    /
        //           '---  X X X X  ----'
        //              kiri-blk  kanan-blk
        //                 (dead zone)
        //
        //   idx   0  = -135°  kanan-belakang   scan_.ranges[0]
        //   idx 180  =  -90°  kanan            scan_.ranges[180]
        //   idx 270  =  -67°  kanan-depan      scan_.ranges[270]
        //   idx 540  =    0°  depan (tengah)   scan_.ranges[540]
        //   idx 810  =  +67°  kiri-depan       scan_.ranges[810]
        //   idx 900  =  +90°  kiri             scan_.ranges[900]
        //   idx 1080 = +135°  kiri-belakang    scan_.ranges[1080]
        //
        // Sudut titik ke-i : scan_.angle_min + i * scan_.angle_increment
        // Index dari sudut : (int)((sudut_rad - scan_.angle_min) / scan_.angle_increment)
        //
        // Cek valid: std::isfinite(scan_.ranges[i]) &&
        //            scan_.ranges[i] >= scan_.range_min &&
        //            scan_.ranges[i] <= scan_.range_max
      });

    web_gui_sub_ = create_subscription<eira_msgs::msg::WebGuiData>(
      "/web_gui_data", 10,
      [this](const eira_msgs::msg::WebGuiData::SharedPtr msg) {
        if (!msg->btn_tictactoe.empty())
          web_btn_tictactoe0_ = msg->btn_tictactoe[0];
      });

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto btn  = [&](int i) -> bool   { return i < (int)msg->buttons.size() && msg->buttons[i]; };
        auto axis = [&](int i) -> double { return i < (int)msg->axes.size() ? (double)msg->axes[i] : 0.0; };

        // nama tombol & axis joystick — sesuaikan index di sini jika ganti joystick
        const bool   btn_pos_ctrl  = btn(0);
        const bool   btn_e_stop    = btn(1);
        const bool   btn_lifter_test = btn(3);
        const double axis_vx       = axis(1);   // stick kiri, sumbu Y
        const double axis_vy       = axis(0);   // stick kiri, sumbu X
        const double axis_wz       = axis(2);   // stick kanan, sumbu X

        // btn_pos_ctrl: toggle position control (idle hold pakai IMU yaw)
        if (btn_pos_ctrl && !prev_joy_btn_[0]) {
          pos_ctrl_active_ = !pos_ctrl_active_;
          if (pos_ctrl_active_) {
            pos_ctrl_yaw_ref_ = imu_yaw_;
          } else {
            stop();
          }
          joy_pickup_req_ = true;   // trigger pick up test di loop
        }

        // btn_e_stop: toggle e_stop — paksa keluar dari semua loop, robot berhenti
        if (btn_e_stop && !prev_joy_btn_[1]) {
          e_stop_ = !e_stop_;
          if (e_stop_) {
            stop();
            for (int i = 0; i < 4; i++) gripper_used_[i] = false;
            RCLCPP_WARN(get_logger(), "=== E-STOP aktif → gripper reset ===");
          }
        }

        // btn_lifter_test: toggle lifterHw antara -300 dan 0
        if (btn_lifter_test && !prev_joy_btn_[3]) {
          joy_lifter_test_active_ = !joy_lifter_test_active_;
          lifterHw(joy_lifter_test_active_ ? -500 : -1000);
        }

        // cmd_vel manual pakai joystick (selalu aktif, bisa override saat e_stop_)
        auto dz = [](double v) { return std::abs(v) < 0.05 ? 0.0 : v; };
        joy_vx_ = dz(axis_vx) * 0.5;
        joy_vy_ = dz(axis_vy) * 0.5;
        joy_wz_ = dz(axis_wz) * 0.5;

        // publish cmd_vel manual HANYA saat e_stop_ aktif
        if (e_stop_) {
          cmdVel(joy_vx_, joy_vy_, joy_wz_);

        }

        for (int i = 0; i < 4; i++) prev_joy_btn_[i] = btn(i);
      });

    // ── publisher hardware ────────────────────────────────────────────────
    cmd_pub_           = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",        10);
    lifter_robot1_pub_ = create_publisher<std_msgs::msg::Int16>("lifter_robot1",       10);
    lifter_robot2_pub_ = create_publisher<std_msgs::msg::Int16>("lifter_robot2",       10);
    dynamixel1_pub_    = create_publisher<std_msgs::msg::Int16>("sudut_dynamixel1",    10);
    penumatic_pub_     = create_publisher<std_msgs::msg::Int16>("penumatic",           10);
    turet_utama_pub_   = create_publisher<std_msgs::msg::Int16>("sudut_turet_utama",   10);
    turet1_pub_        = create_publisher<std_msgs::msg::Int16>("sudut_turet1",        10);
    turet2_pub_        = create_publisher<std_msgs::msg::Int16>("sudut_turet2",        10);
    lifter_turet1_pub_ = create_publisher<std_msgs::msg::Int16>("lifter_turet1",       10);
    lifter_turet2_pub_ = create_publisher<std_msgs::msg::Int16>("lifter_turet2",       10);
    griper1_pub_       = create_publisher<std_msgs::msg::Int16>("griper1",             10);
    griper2_pub_       = create_publisher<std_msgs::msg::Int16>("griper2",             10);
    griper3_pub_       = create_publisher<std_msgs::msg::Int16>("griper3",             10);
    griper4_pub_       = create_publisher<std_msgs::msg::Int16>("griper4",             10);
    lifter_base_belakang_pub_ = create_publisher<std_msgs::msg::Int16>("lifter_base_belakang", 10);

    loop_thread_ = std::thread([this]{ loop(); });

  }

  ~SeqNode()
  {
    running_ = false;
    if (loop_thread_.joinable()) loop_thread_.join();
  }

private:
  // ════════════════════════════════════════════════════════════════════════
  //  HELPER
  // ════════════════════════════════════════════════════════════════════════

  void delay(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

  static float clamp(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

  // yaw relatif terhadap saat masuk seqForest (dianggap 0°)
  float forestYaw() const {
    float y = imu_yaw_ - forest_yaw_offset_;
    if (y >  180.f) y -= 360.f;
    if (y < -180.f) y += 360.f;
    return y;
  }

  void cmdVel(double vx, double vy, double wz)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = vx; t.linear.y = vy; t.angular.z = wz;
    cmd_pub_->publish(t);
  }

  void stop() { cmdVel(0, 0, 0); }

  void lifterBaseBelakang(int16_t val)
  {
    std_msgs::msg::Int16 m; m.data = val;
    lifter_base_belakang_pub_->publish(m);
  }

  void lifterHw(int16_t val)   // robot1 & 2 sekaligus
  {
    std_msgs::msg::Int16 m; m.data = val;
    lifter_robot1_pub_->publish(m);
    lifter_robot2_pub_->publish(m);
  }

  void lifterRobot(int id, int16_t val)
  {
    std_msgs::msg::Int16 m; m.data = val;
    if (id == 1) lifter_robot1_pub_->publish(m);
    else         lifter_robot2_pub_->publish(m);
  }

  void griper(int id, int16_t val)
  {
    std_msgs::msg::Int16 m; m.data = val;
    switch (id) {
      case 1: griper1_pub_->publish(m); break;
      case 2: griper2_pub_->publish(m); break;
      case 3: griper3_pub_->publish(m); break;
      case 4: griper4_pub_->publish(m); break;
    }
  }

  void sudutTuret(int id, int16_t val)
  {
    std_msgs::msg::Int16 m; m.data = val;
    if (id == 1) turet1_pub_->publish(m);
    else         turet2_pub_->publish(m);
  }

  void turetUtama(int16_t val)
  { std_msgs::msg::Int16 m; m.data = val; turet_utama_pub_->publish(m); }

  void lifterTuret(int id, int16_t val)
  {
    std_msgs::msg::Int16 m; m.data = val;
    if (id == 1) lifter_turet1_pub_->publish(m);
    else         lifter_turet2_pub_->publish(m);
  }

  void dynamixel(int16_t val)
  { std_msgs::msg::Int16 m; m.data = val; dynamixel1_pub_->publish(m); }

  void penumatic(int16_t val)
  { std_msgs::msg::Int16 m; m.data = val; penumatic_pub_->publish(m); }

  // tunggu hingga gui_start_ = true
  void waitForStart()
  {
    while (rclcpp::ok() && running_ && !e_stop_ && !gui_start_) {
    }
  }

  // tunggu hingga gui_start_ = false (tombol dilepas)
  void waitForRelease()
  {
    while (rclcpp::ok() && running_ && !e_stop_ && gui_start_) delay(50);
  }

  // ════════════════════════════════════════════════════════════════════════
  //  SPEARHEAD SEQUENCE  (dipindah dari master_node)
  // ════════════════════════════════════════════════════════════════════════

  void seqSpearhead()
  {
    int   sp   = gui_spearhead_;
    int   team = gui_team_;
    float yaw_ref = imu_yaw_;

    // ── fase 1: kirim lifter -400 lalu maju ──────────────────────────────
    lifterHw(-400);

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto ms = [&]{ return std::chrono::duration_cast<std::chrono::milliseconds>(
                     clock::now() - t0).count(); };

    while (rclcpp::ok() && running_ && !e_stop_) {
      long t = ms();
      if      (t < 100) { /* tunggu */ }
      else if (t < 700) { cmdVel(0.5, 0, 0); }
      else if (t < 900) { cmdVel(0.2, 0, 0); }
      else              { break; }
      delay(10);
    }
    // stop();
    lifterRobot(1, -898);
    lifterRobot(2, -900);

    // ── fase 2: koreksi TOF (depan + lateral) + yaw ──────────────────────
    bool is_merah = (team == 1);

    while (rclcpp::ok() && running_ && !e_stop_) {
      float avg_front, err_fwd, target_lat, tof_lat_mm, err_lat;
      bool  tof_lat_valid;

      if (is_merah) {
        if (tof_s2_ == 0) { delay(20); continue; }
        avg_front    = (float)tof_s2_;
        err_fwd      = avg_front - 115.f;
        target_lat   = (sp >= 1 && sp <= 6) ? kSpearMerah[sp] : 0.f;
        tof_lat_mm   = (tof_s3_ > 0) ? (float)tof_s3_ : -1.f;
        tof_lat_valid= (tof_lat_mm > 0.f);
        err_lat      = tof_lat_valid ? (tof_lat_mm - target_lat) : 0.f;
      } else {
        if (tof_s1_ == 0) { delay(20); continue; }
        avg_front    = (float)tof_s1_;
        err_fwd      = avg_front - 115.f;
        target_lat   = (sp >= 1 && sp <= 6) ? kSpearBiru[sp] : 0.f;
        tof_lat_mm   = (tof_s4_ > 0) ? (float)tof_s4_ : -1.f;
        tof_lat_valid= (tof_lat_mm > 0.f);
        err_lat      = tof_lat_valid ? (target_lat - tof_lat_mm) : 0.f;
      }

      float err_yaw = imu_yaw_ - yaw_ref;
      float out_fwd = clamp(kTofKp * err_fwd, kTofMax);
      float lat_max = (std::fabs(err_lat) < kWallSlow) ? 0.1f : 0.2f;
      float out_lat = tof_lat_valid ? clamp(kWallKp * err_lat, lat_max) : 0.f;
      float out_wz  = clamp(kPosKpYaw * err_yaw, kPosMaxWz);

      bool done_fwd = std::fabs(err_fwd) < kTolFwd;
      bool done_lat = !tof_lat_valid || std::fabs(err_lat) < kWallTol;

      if (done_fwd && done_lat) {
        penumatic(1);
        dynamixel(kDyn1Ambil);
        break;
      }

      cmdVel(done_fwd ? 0.0 : (double)out_fwd,
             done_lat ? 0.0 : (double)out_lat,
             (double)out_wz);
      delay(20);
    }
    stop();

    // ── fase 3: POST_SEQUENCE (dynamixel setelah posisi) ─────────────────
    delay(600);
    penumatic(0);           // t=0.5s
    delay(500);
    lifterHw(-1100);        // t=1.0s
    delay(500);
    dynamixel(kDyn1Atas);   // t=1.5s
    delay(200);
    lifterHw(-500);        // t=1.0s

    // ── fase 4: mundur ke target TOF ─────────────────────────────────────
    float rotate_target_yaw = 0.f;

    while (rclcpp::ok() && running_ && !e_stop_) {
      if (is_merah) {
        if (tof_s2_ == 0) { delay(20); continue; }
        float err_vx = 1000.f - (float)tof_s2_;
        float out_vx = clamp(-kRevKp * err_vx, kRevMaxVx);
        float err_vy = (tof_s3_ > 0) ? ((float)tof_s3_ - 250.f) : 0.f;
        float out_vy = clamp(kWallKp * err_vy, 0.1f);
        bool done_vx = std::fabs(err_vx) < kRevTol;
        bool done_vy = tof_s3_ == 0 || std::fabs(err_vy) < 15.f;
        if (done_vx && done_vy) {
          rotate_target_yaw = imu_yaw_ + 90.f;
          if (rotate_target_yaw > 180.f) rotate_target_yaw -= 360.f;
          break;
        }
        cmdVel(done_vx ? 0.0 : (double)out_vx, (double)out_vy, 0.0);
      } else {
        if (tof_s1_ == 0) { delay(20); continue; }
        float err_vx = 900 - (float)tof_s1_;
        float out_vx = clamp(-kRevKp * err_vx, kRevMaxVx);
        float err_vy = (tof_s4_ > 0) ? (250.f - (float)tof_s4_) : 0.f;
        float out_vy = clamp(kWallKp * err_vy, 0.1f);
        bool done_vx = std::fabs(err_vx) < kRevTol;
        bool done_vy = tof_s4_ == 0 || std::fabs(err_vy) < 15.f;
        if (done_vx && done_vy) {
          rotate_target_yaw = imu_yaw_ - 90.f;
          if (rotate_target_yaw < -180.f) rotate_target_yaw += 360.f;
          break;
        }
        cmdVel(done_vx ? 0.0 : (double)out_vx, (double)out_vy, 0.0);
      }
      delay(20);
    }
    stop();

    // ── fase 5: putar 90° IMU ─────────────────────────────────────────────
    while (rclcpp::ok() && running_ && !e_stop_) {
      float err_yaw = imu_yaw_ - rotate_target_yaw;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      float out_wz = clamp(0.02f * err_yaw, 0.5f);
      if (std::fabs(err_yaw) < 2.0f) {
        stop();
        lifterHw(-10);
        break;
      }
      cmdVel(0.0, 0.0, (double)out_wz);
      delay(20);
    }
    delay(500);
    cmdVel(-0.2, 0.0, 0.0);
    delay(500);
    stop();

  }

  // ════════════════════════════════════════════════════════════════════════
  //  FOREST / KFS SEQUENCE
  //
  //  Contoh plan dari intelligent_node (ros2 topic echo /kfs_retrieval_plan):
  //
  //  step_count: 5
  //  steps:
  //   - step: 0  action: 4  action_name: ROTATE   heading: 90   direction: "-"   elev: "-"   delta: 0
  //   - step: 1  action: 1  action_name: APPROACH  heading: 90   direction: FORWARD  elev: UP   delta: 800
  //   - step: 2  action: 2  action_name: ENTER     heading: 90   direction: FORWARD  elev: FLAT  delta: 0
  //   - step: 5  action: 5  action_name: PICK_UP   target: 3     direction: "-"   elev: FLAT  delta: 0
  //   - step: 3  action: 7  action_name: EXIT      heading: 90   direction: BACKWARD elev: "-"  delta: 0
  //
  //  Artinya: putar 90° → maju angkat lifter 800 → masuk pelan sampai TOF →
  //           ambil KFS #3 → mundur keluar
  // ════════════════════════════════════════════════════════════════════════

  // direction string → (vx, vy)
  static void dirToVel(const std::string & dir, float spd, float & vx, float & vy)
  {
    vx = vy = 0.f;
    if      (dir == "FORWARD"  || dir == "Forward")  vx =  spd;
    else if (dir == "BACKWARD" || dir == "Backward") vx = -spd;
    else if (dir == "LEFT"     || dir == "Left")     vy =  spd;
    else if (dir == "RIGHT"    || dir == "Right")    vy = -spd;
  }

  // reposisi setelah ENTER target 2: wz dari tof_s2-tof_s1, vx dari tof_s4 target 370mm
  // setelah selesai simpan imu_yaw_ sebagai referensi heading baru
void reposisiF1()
  {
    while (rclcpp::ok() && running_ && !e_stop_) {
      if (tof_s1_ == 0 || tof_s2_ == 0 || tof_s4_ == 0) { delay(20); continue; }

      // wz: paralel dinding dari selisih tof_s2 - tof_s1
      float diff_tof = (float)tof_s2_ - (float)tof_s1_;
      float out_wz   = clamp(kApproachKpWz * diff_tof, kApproachMaxWz);

      // vx: rata-rata tof_s1 dan tof_s2, target 300mm
      float avg_front = ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
      float err_fwd   = avg_front - kApproachTargetFront;
      float out_vx    = clamp(kApproachKpVx * err_fwd, kApproachMaxVx);
      bool  done_fwd  = std::fabs(err_fwd) < kApproachTolVx;

      // vy: tof_s4 (kanan), target 370mm
      float err_s4   = 370.f - (float)tof_s4_;
      float out_vy   = clamp(kApproachKpVy * err_s4, kApproachMaxVy);
      bool  done_lat = std::fabs(err_s4) < kApproachTolVy;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "reposisiF2 | s4=%.0f err_vy=%.0f out_vy=%.3f | avg=%.0f err_vx=%.0f out_vx=%.3f | diff=%.0f | %s%s",
        (float)tof_s4_, err_s4, out_vy, avg_front, err_fwd, out_vx, diff_tof,
        done_fwd ? "VX✓ " : "VX.. ", done_lat ? "VY✓" : "VY..");

      if (done_fwd) { stop(); yaw_ref_ = forestYaw(); break; }

      cmdVel(done_fwd ? 0.0 : (double)out_vx,
             0.0,
             (double)out_wz);
      delay(20);
    }
  }


  void reposisiF2()
  {
    while (rclcpp::ok() && running_ && !e_stop_) {
      if (tof_s1_ == 0 || tof_s2_ == 0 || tof_s4_ == 0) { delay(20); continue; }

      // wz: paralel dinding dari selisih tof_s2 - tof_s1
      float diff_tof = (float)tof_s2_ - (float)tof_s1_;
      float out_wz   = clamp(kApproachKpWz * diff_tof, kApproachMaxWz);

      // vx: rata-rata tof_s1 dan tof_s2, target 300mm
      float avg_front = ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
      float err_fwd   = avg_front - kApproachTargetFront;
      float out_vx    = clamp(kApproachKpVx * err_fwd, kApproachMaxVx);
      bool  done_fwd  = std::fabs(err_fwd) < kApproachTolVx;

      // vy: tof_s4 (kanan), target 370mm
      float err_s4   = 370.f - (float)tof_s4_;
      float out_vy   = clamp(kApproachKpVy * err_s4, kApproachMaxVy);
      bool  done_lat = std::fabs(err_s4) < kApproachTolVy;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "reposisiF2 | s4=%.0f err_vy=%.0f out_vy=%.3f | avg=%.0f err_vx=%.0f out_vx=%.3f | diff=%.0f | %s%s",
        (float)tof_s4_, err_s4, out_vy, avg_front, err_fwd, out_vx, diff_tof,
        done_fwd ? "VX✓ " : "VX.. ", done_lat ? "VY✓" : "VY..");

      if (done_fwd && done_lat) { stop(); yaw_ref_ = forestYaw(); break; }

      cmdVel(done_fwd ? 0.0 : (double)out_vx,
             done_lat ? 0.0 : (double)out_vy,
             (double)out_wz);
      delay(20);
    }
  }

  // reposisi umum berdasarkan wall[] dari plan intelligent
  //   wall[0]=front → avg(s1,s2) target 300mm (vx)
  //   wall[1]=left  → tof_s3 target 370mm (vy-)
  //   wall[2]=right → tof_s4 target 370mm (vy+)
  //   wz selalu dari selisih tof_s2 - tof_s1
  void reposisiStep(const ForestPlanStep & step)
  {
    bool use_front = step.wall.size() > 0 && step.wall[0] != 0;
    bool use_left  = step.wall.size() > 1 && step.wall[1] != 0;
    bool use_right = step.wall.size() > 2 && step.wall[2] != 0;
    // lifterHw(-500);
    // delay(500);

    if (!use_front && !use_left && !use_right) return;

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto ms = [&]{ return std::chrono::duration_cast<std::chrono::milliseconds>(
                     clock::now() - t0).count(); };

    while (rclcpp::ok() && running_ && !e_stop_ && ms() < 5000) {
      if (tof_s1_ == 0 || tof_s2_ == 0) { delay(20); continue; }

      // wz
      float diff_tof = (float)tof_s2_ - (float)tof_s1_;
      float out_wz   = clamp(kApproachKpWz * diff_tof, kApproachMaxWz);

      // vx
      float out_vx   = 0.f;
      bool  done_fwd = true;
      if (use_front) {
        float avg    = ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
        float err_vx = avg - 300.f;
        out_vx   = clamp(kApproachKpVx * err_vx, kApproachMaxVx);
        done_fwd = std::fabs(err_vx) < kApproachTolVx;
      }

      // vy
      float out_vy   = 0.f;
      bool  done_lat = true;
      if (use_right && tof_s4_ > 0) {
        float err = 370.f - (float)tof_s4_;
        out_vy   = clamp(kApproachKpVy * err, kApproachMaxVy);
        done_lat = std::fabs(err) < kApproachTolVy;
      } else if (use_left && tof_s3_ > 0) {
        float err = (float)tof_s3_ - 370.f;
        out_vy   = clamp(kApproachKpVy * err, kApproachMaxVy);
        done_lat = std::fabs(err) < kApproachTolVy;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "reposisi | vx=%.3f vy=%.3f wz=%.3f | %s%s",
        out_vx, out_vy, out_wz,
        done_fwd ? "VX✓ " : "VX.. ", done_lat ? "VY✓" : "VY..");

      if (done_fwd && done_lat) { stop(); break; }

      cmdVel(done_fwd ? 0.0 : (double)out_vx,
             done_lat ? 0.0 : (double)out_vy,
             (double)out_wz);
      delay(20);
    }
  }

  // kontrol posisi ke F1/F2/F3 pakai TOF + wall, inputan forest = 1/2/3
  // catatan: logic ini di-tune untuk tim BIRU (wall_kanan_) — lihat memory JALAN_FOREST.
  void posisiForest(int forest)
  {
    bool  is_merah = (gui_team_ == 1);
    float wall_tgt = (forest >= 1 && forest <= 3)
                     ? (is_merah ? kForestWallMerah[forest] : kForestWallBiru[forest])
                     : 0.f;
    float yaw_ref = forestYaw();
    RCLCPP_INFO(get_logger(), "posisiForest F%d %s wall_tgt=%.0fmm",
      forest, is_merah ? "MERAH" : "BIRU", wall_tgt);

    while (rclcpp::ok() && running_ && !e_stop_) {
      if (tof_s1_ == 0 || tof_s2_ == 0) { delay(20); continue; }

      // vx: rata-rata tof depan, target 300mm
      float avg_front = ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
      float err_fwd   = avg_front - kApproachTargetFront;
      float out_vx    = clamp(kApproachKpVx * err_fwd, kApproachMaxVx);
      bool  done_fwd  = std::fabs(err_fwd) < kApproachTolVx;

      // wz: pakai tof_s1 vs tof_s2 jika keduanya < 700mm, selainnya imu_yaw
      float out_wz;
      if (tof_s1_ > 0 && tof_s1_ < 700 && tof_s2_ > 0 && tof_s2_ < 700) {
        float diff_tof = (float)tof_s2_ - (float)tof_s1_;
        out_wz = clamp(kApproachKpWz * diff_tof, kApproachMaxWz);
      } else {
        float err_yaw = forestYaw() - yaw_ref;
        if (err_yaw >  180.f) err_yaw -= 360.f;
        if (err_yaw < -180.f) err_yaw += 360.f;
        out_wz = clamp(kApproachKpWz * err_yaw, kApproachMaxWz);
      }

      // vy: wall_kanan (biru) / wall_kiri (merah), target dalam mm
      float wall_now  = is_merah ? wall_kiri_ : wall_kanan_;
      float out_vy    = 0.f;
      bool  done_lat  = true;
      if (wall_now > 0.f && wall_tgt > 0.f) {
        float err_wall = wall_now - wall_tgt;
        float lat_max  = std::fabs(err_wall) < kApproachSlowVy ? 0.05f : kApproachMaxVy;
        float sign     = is_merah ? 1.f : -1.f;
        out_vy   = sign * clamp(kApproachKpVy * err_wall, lat_max);
        done_lat = std::fabs(err_wall) < kApproachTolVy;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "F%d | wall_now=%.0f tgt=%.0f err=%.0f out_vy=%.3f | fwd_err=%.0f out_vx=%.3f | %s%s",
        forest, wall_now, wall_tgt, wall_now - wall_tgt, out_vy,
        err_fwd, out_vx,
        done_fwd ? "VX✓ " : "VX.. ", done_lat ? "VY✓" : "VY..");

      if (done_fwd && done_lat) { stop(); break; }

      cmdVel(done_fwd ? 0.0 : (double)out_vx,
             done_lat ? 0.0 : (double)out_vy,
             (double)out_wz);
      delay(20);
    }
  }

  // ACTION_PICK_UP (5):
  //   Urutan: 1 → 3 → (putar turetUtama -760 sekali) → 2 → 4
  //   turetUtama hanya diputar saat pertama kali mau pakai gripper 2
  void stepPickUp(const ForestPlanStep & step)
  {
    // kirim lifterHw sesuai elev sebelum maju
    if (step.elev == "DOWN") {
      lifterHw(-50);
      delay(1000);
    } else if (step.elev == "UP") {
      int16_t pos = (step.delta == 400) ? kElevDelta400 : kElevDelta200;
      lifterHw(pos);
      delay(800);
    }

    // maju sebelum pickup: DOWN = sampai mid1 > 100, selainnya sampai mid1 < 100
    {
      float yaw_ref = forestYaw();
      bool  is_down = (step.elev == "DOWN");
      while (rclcpp::ok() && running_ && !e_stop_) {
        bool trigger = is_down ? (tof_mid1_ > 100 && tof_mid1_2_ > 100)
                                : (tof_mid1_ > 0 && tof_mid1_ < 100);
        if (trigger) { stop(); break; }
        float err_yaw = forestYaw() - yaw_ref;
        if (err_yaw >  180.f) err_yaw -= 360.f;
        if (err_yaw < -180.f) err_yaw += 360.f;
        if (is_down) {
          float wz = (std::abs(err_yaw) > 1.0f) ? clamp(kMoveKpWz * err_yaw, kMoveMaxWz) : 0.0f;
          cmdVel(0.08, 0.0, wz);
        }
        else
        {
          cmdVel(kMoveSpeed, 0.0, clamp(kMoveKpWz * err_yaw, kMoveMaxWz));
        }
        delay(20);
      }
    }

    // urutan pickup: 1, 3, 2, 4
    static const int order[4] = {1, 3, 4, 2};
    int g = -1;
    for (int i = 0; i < 4; i++) {
      if (!gripper_used_[order[i] - 1]) { g = order[i]; break; }
    }
    if (g == -1) {
      RCLCPP_WARN(get_logger(), "  PICK_UP: semua gripper sudah penuh!");
      return;
    }

    // putar turetUtama sekali saat pertama kali masuk ke gripper 2/4
    if (g == 4 && !turet_rotated_) {
      turetUtama(770);
      delay(2500);
      turet_rotated_ = true;
    }

    switch (g) {

      case 1:
        if (step.elev == "UP") {
        // lifterTuret(1, -1000);
        // sudutTuret(1, 630);
        // delay(500);
        griper(1, 1);
        lifterTuret(1, -3000);
        delay(1000);

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s6 = tof_gripper_[6];
            int32_t s7 = tof_gripper_[7];
            if (s6 > 350 && s7 > 350) { stop(); break; }
            if      (s6 > 400) last_vy = -0.11f;
            else if (s6 < 300) last_vy =  0.11f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }

        sudutTuret(1, 820);
        delay(1000);
        griper(1, 0);
        delay(1000);
        sudutTuret(1, 0);
        delay(500);
        lifterTuret(1, -1000);
        delay(2000);
      }
      else 
      {
        lifterTuret(1, -1000);
        sudutTuret(1, 820);
        delay(500);
        griper(1, 1);
        lifterTuret(1, -3000);
        delay(1000);

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s6 = tof_gripper_[6];
            int32_t s7 = tof_gripper_[7];
            if (s6 > 350 && s7 > 350) { stop(); break; }
            if      (s6 > 400) last_vy = -0.11f;
            else if (s6 < 300) last_vy =  0.11f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }

        sudutTuret(1, 1000);
        delay(1000);
        griper(1, 0);
        delay(1000);
        sudutTuret(1, 0);
         delay(500);
        lifterTuret(1, -1000);
        delay(2000);

      }
        break;

      case 3:
        if (step.elev == "UP") {
        lifterTuret(2, -1000);
        sudutTuret(2, 660);
        delay(500);
        griper(3, 1);
        lifterTuret(2, -3000);
        delay(1000);

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s2 = tof_gripper_[3];
            int32_t s3 = tof_gripper_[2];
            if (s2 > 350 && s3 > 350) { stop(); break; }
            if      (s3 > 400) last_vy = -0.11f;
            else if (s3 < 300) last_vy =  0.11f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }

        sudutTuret(2, 820);
        delay(1000);
        griper(3, 0);
        delay(1000);
        sudutTuret(2, 0);
         delay(500);
        lifterTuret(2, -1000);
        delay(2000);
      }
      else 
      {
        lifterTuret(2, -1000);
        sudutTuret(2, 820);
        delay(500);
        griper(3, 1);
        lifterTuret(2, -3000);
        delay(1000);

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s2 = tof_gripper_[3];
            int32_t s3 = tof_gripper_[2];
            if (s2 > 350 && s3 > 350) { stop(); break; }
            if      (s3 > 400) last_vy = -0.11f;
            else if (s3 < 300) last_vy =  0.11f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }

        sudutTuret(2, 1000);
        delay(1000);
        griper(3, 0);
        delay(1000);
        sudutTuret(2, 0);
         delay(500);
        lifterTuret(2, -1000);
        delay(2000);

      }
        break;

      case 2:
        if (step.elev == "UP") {      
        lifterTuret(1, -1000);
        sudutTuret(1, -630);
        delay(500);
        griper(2, 1);
        lifterTuret(1, -3000);
        delay(1000);

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s4 = tof_gripper_[5];
            int32_t s5 = tof_gripper_[4];
            if (s4 > 350 && s5 > 350) { stop(); break; }
            if      (s5 > 400) last_vy = -0.11f;
            else if (s5 < 300) last_vy =  0.11f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }

        sudutTuret(1, -820);
        delay(1000);
        griper(2, 0);
        delay(1000);
        sudutTuret(1, 0);
         delay(500);
        lifterTuret(1, -1000);
        delay(2000);

      }
      else 
      {
        lifterTuret(1, -1000);
        sudutTuret(1, -820);
        delay(500);
        griper(2, 1);
        lifterTuret(1, -3000);
        delay(1000);

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s4 = tof_gripper_[5];
            int32_t s5 = tof_gripper_[4];
            if (s4 > 350 && s5 > 350) { stop(); break; }
            if      (s5 > 400) last_vy = -0.11f;
            else if (s5 < 300) last_vy =  0.11f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }

        sudutTuret(1, -1000);
        delay(1000);
        griper(2, 0);
        delay(1000);
        sudutTuret(1, 0);
         delay(500);
        lifterTuret(1, -1000);
        delay(2000);
      }
        break;

      case 4:
        if (step.elev == "UP") {        
        lifterTuret(2, -1000);
        sudutTuret(2, -630);
        delay(500);
        griper(4, 1);
        lifterTuret(2, -3000);
        lifterTuret(2, -3000);
        delay(1000);

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s0 = tof_gripper_[1];
            int32_t s1 = tof_gripper_[0];
            if (s0 > 350 && s1 > 350) { stop(); break; }
            if      (s0 > 400) last_vy = -0.11f;
            else if (s0 < 300) last_vy =  0.11f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }

        sudutTuret(2, -820);
        delay(1000);
        griper(4, 0);
        delay(1000);
        sudutTuret(2, 0);
         delay(500);
        lifterTuret(2, -1000);
        delay(2000);
      }
      else 
      {
        lifterTuret(2, -1000);
        sudutTuret(2, -820);
        delay(500);
        griper(4, 1);
        lifterTuret(2, -3000);
        delay(1000);

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s0 = tof_gripper_[1];
            int32_t s1 = tof_gripper_[0];
            if (s0 > 350 && s1 > 350) { stop(); break; }
            if      (s0 > 400) last_vy = -0.11f;
            else if (s0 < 300) last_vy =  0.11f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }
  
        sudutTuret(2, -1000);
        delay(1000);
        griper(4, 0);
        delay(1000);
        sudutTuret(2, 0);
         delay(500);
        lifterTuret(2, -1000);
        delay(2000);

      }
        break;
    }

    gripper_used_[g - 1] = true;
  }

  // ACTION_PLACE_BOX (tictactoe): kebalikan pick up — cari gripper terisi,
  //   posisikan robot, lalu BUKA gripper untuk release box.
  //
  //   Target scan kiri BERBEDA per kelompok lifter turet:
  //
  //   Gripper 1,2  (lifter turet 1):      Gripper 3,4  (lifter turet 2):
  //     rak 1 → 500mm                       rak 1 → 900mm
  //     rak 2 → 1000mm                      rak 2 → 1400mm  (900+500)
  //     rak 3 → 1500mm                      rak 3 → 1900mm  (900+1000)
  //
  //   Sebelum bergerak: lifterHw(-700)
  //   Posisi:  vx = avg(s1,s2) → 600mm   (kApproachKpVx / kPlaceMaxVx=0.3)
  //            vy = scan kiri  → target   (kScanKpVy     / kPlaceMaxVy=0.3)
  //            wz = diff(s2-s1)           (kApproachKpWz / kApproachMaxWz)
  void stepPlaceBox(int rak)
  {
    // ── tentukan gripper dari gripper_used_ (diisi oleh stepPickUp) ────
    // urutan: 1 → 3 → 2 → 4  (sama seperti pick up)
    static constexpr int kPlaceOrder[4] = {1, 3, 4, 2};
    int g = -1;
    for (int i = 0; i < 4; i++) {
      if (gripper_used_[kPlaceOrder[i] - 1]) { g = kPlaceOrder[i]; break; }
    }
    if (g == -1) {
      RCLCPP_WARN(get_logger(), "  PLACE_BOX: tidak ada gripper terisi!");
      return;
    }

    // ── pilih target scan kiri berdasarkan gripper ───────────────────────
    // gripper 1,2 → lifter turet 1 (lebih depan)
    // gripper 3,4 → lifter turet 2 (lebih belakang, offset +400mm)
    static constexpr float kPlaceTgtG12[4] = {0.f,  450.f, 970.f, 600.f};
    static constexpr float kPlaceTgtG34[4] = {0.f,  850.f, 1400.f, 1900.f};
    bool  use_g12     = (g == 1 || g == 2);
    bool  use_kanan   = (rak == 3);   // rak 3 (pos 6,9): pakai scan kanan
    float target_scan = 0.f;
    if (use_kanan) {
      target_scan = use_g12 ? kPlaceRak3KananG12 : kPlaceRak3KananG34;
    } else {
      target_scan = (rak >= 1 && rak <= 3)
        ? (use_g12 ? kPlaceTgtG12[rak] : kPlaceTgtG34[rak])
        : (use_g12 ? kPlaceTgtG12[1]   : kPlaceTgtG34[1]);
    }

    RCLCPP_INFO(get_logger(), "=== PLACE_BOX rak=%d gripper=%d (%s) scan_%s=%.0fmm ===",
      rak, g, use_g12 ? "turet1" : "turet2", use_kanan ? "kanan" : "kiri", target_scan);

    // ── mundur 500mm sebelum geser ke rak (skip saat pertama kali) ──────
    if (tictactoe_place_idx_ > 0) {
      float yaw_mundur500 = forestYaw();
      float target_mundur = (tof_s1_ > 0) ? ((float)tof_s1_ + 500.f) : 800.f;
      RCLCPP_INFO(get_logger(), "  PLACE_BOX: mundur 500mm → s1 target=%.0fmm", target_mundur);
      while (rclcpp::ok() && running_ && !e_stop_) {
        if (tof_s1_ == 0) { delay(20); continue; }
        float err_vx = (float)tof_s1_ - target_mundur;
        if (std::fabs(err_vx) < kApproachTolVx) { stop(); break; }
        float err_yaw = forestYaw() - yaw_mundur500;
        if (err_yaw >  180.f) err_yaw -= 360.f;
        if (err_yaw < -180.f) err_yaw += 360.f;
        cmdVel((double)clamp(kApproachKpVx * err_vx, kPlaceMaxVx),
               0.0,
               (double)clamp(kMoveKpWz * err_yaw, kMoveMaxWz));
        delay(20);
      }
    }

    // ── sebelum pindah sisi: mundur vx sampai tof s1 = 1000mm ─────────
    if (g == 4) {
      // mundur vx → s1=1000mm, sambil vy ke tengah rak 2 (scan kiri)
      float tgt_rak2_kiri = use_g12 ? kPlaceTgtG12[2] : kPlaceTgtG34[2];
      RCLCPP_INFO(get_logger(), "  PLACE_BOX: mundur → s1=1000mm + vy rak2=%.0fmm", tgt_rak2_kiri);
      float yaw_mundur  = forestYaw();
      float scan_ema_g2 = -1.f;   // EMA state untuk loop mundur
      while (rclcpp::ok() && running_ && !e_stop_) {
        if (tof_s1_ == 0) { delay(20); continue; }

        // vx: mundur sampai s1 = 1000mm
        float err_vx = (float)tof_s1_ - 1000.f;
        bool  done_vx = std::fabs(err_vx) < kApproachTolVx;

        // vy: scan kiri → rak 2 center  (median 20 titik + EMA α=0.5)
        int   n  = (int)scan_.ranges.size();
        float sm = 0.f;
        int   vc = 0;
        if (n > 0) {
          float buf[20]; int bc = 0;
          int ic = n - 1;
          for (int i = ic - 19; i <= ic; i++) {
            if (i < 0 || i >= n || bc >= 20) continue;
            float r = scan_.ranges[i];
            if (std::isfinite(r) && r >= scan_.range_min && r <= scan_.range_max)
              buf[bc++] = r * 1000.f;
          }
          if (bc > 0) {
            std::sort(buf, buf + bc);
            float median = buf[bc / 2];
            scan_ema_g2 = (scan_ema_g2 < 0.f) ? median : (0.5f * median + 0.5f * scan_ema_g2);
            sm = scan_ema_g2;
            vc = bc;
          }
        }
        float out_vy = 0.f;
        bool  done_vy = false;
        if (vc > 0) {
          float err_vy = tgt_rak2_kiri - sm;
          out_vy  = clamp(kScanKpVy * err_vy, kPlaceMaxVy);
          done_vy = std::fabs(err_vy) < kScanTolVy;
        }

        // wz: IMU yaw hold
        float err_yaw = forestYaw() - yaw_mundur;
        if (err_yaw >  180.f) err_yaw -= 360.f;
        if (err_yaw < -180.f) err_yaw += 360.f;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
          "  mundur: s1=%.0fmm vx=%.3f | kiri=%.0fmm vy=%.3f | %s%s",
          (float)tof_s1_, clamp(kApproachKpVx * err_vx, kPlaceMaxVx),
          sm, out_vy, done_vx ? "VX✓ " : "VX.. ", done_vy ? "VY✓" : "VY..");

        if (done_vx && done_vy) { stop(); break; }

        cmdVel(done_vx ? 0.0 : (double)clamp(kApproachKpVx * err_vx, kPlaceMaxVx),
               done_vy ? 0.0 : (double)out_vy,
               (double)clamp(kMoveKpWz * err_yaw, kMoveMaxWz));
        delay(20);
      }

      // ── putar turetUtama 180° ────────────────────────────────────────
      RCLCPP_INFO(get_logger(), "  PLACE_BOX: turetUtama -780 (pindah ke sisi 2/4)");
    
      turetUtama(770);
      delay(2500);
    }

    // ── angkat lifter sebelum bergerak ───────────────────────────────────
    lifterHw(-600);
    // if(g == 1)lifterTuret(1, -3000);
    // if(g == 2)lifterTuret(2, -3000);
    // if(g == 3)lifterTuret(3, -3000);
    // if(g == 4)lifterTuret(4, -3000);
    delay(1000);

    // ── posisi: vx dari avg(s1,s2), vy dari scan kiri, wz dari IMU yaw ──
    float yaw_ref  = forestYaw();
    float scan_ema = -1.f;   // EMA state untuk loop positioning
    while (rclcpp::ok() && running_ && !e_stop_) {
      if (tof_s1_ == 0 || tof_s2_ == 0) { delay(20); continue; }

      // vx sensor:
      //   rak1 + gripper 1/2 → s1
      //   gripper 3 + rak 3  → s2
      //   selainnya           → avg(s1,s2)
      float vx_src  = (rak == 1 && use_g12) ? (float)tof_s1_
                    : (g == 3 && rak == 3)   ? (float)tof_s2_
                    :                          ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
      float err_vx  = vx_src - 170.f;
      float out_vx  = clamp(kApproachKpVx * err_vx, kPlaceMaxVx);
      bool  done_vx = std::fabs(err_vx) < kApproachTolVx;

      // wz: diff(s2-s1) atau IMU yaw tergantung gripper dan rak
      //   gripper 1,4 → diff(s2-s1) di rak 2 dan rak 3
      //   gripper 2,3 → diff(s2-s1) di rak 1 dan rak 2
      //   selainnya   → IMU yaw hold
      bool  use_diff_wz = ((g == 1 || g == 4) && (rak == 2 || rak == 3))
                       || ((g == 2 || g == 3) && (rak == 1 || rak == 2));
      float err_yaw = forestYaw() - yaw_ref;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      float out_wz;
      if (use_diff_wz) {
        float diff_tof = (float)tof_s2_ - (float)tof_s1_;
        out_wz = clamp(kApproachKpWz * diff_tof, kApproachMaxWz);
      } else {
        out_wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
      }

      // vy: scan kiri (rak 1,2) atau scan kanan (rak 3)
      // filter: median dari 20 titik terdekat + EMA α=0.5
      int   n           = (int)scan_.ranges.size();
      float scan_mm     = 0.f;
      int   valid_count = 0;
      if (n > 0) {
        float buf[20]; int bc = 0;
        if (use_kanan) {
          for (int i = 0; i <= 19 && i < n; i++) {
            float r = scan_.ranges[i];
            if (std::isfinite(r) && r >= scan_.range_min && r <= scan_.range_max && bc < 20)
              buf[bc++] = r * 1000.f;
          }
        } else {
          int ic = n - 1;
          for (int i = ic - 19; i <= ic; i++) {
            if (i < 0 || i >= n || bc >= 20) continue;
            float r = scan_.ranges[i];
            if (std::isfinite(r) && r >= scan_.range_min && r <= scan_.range_max)
              buf[bc++] = r * 1000.f;
          }
        }
        if (bc > 0) {
          std::sort(buf, buf + bc);
          float median = buf[bc / 2];
          scan_ema  = (scan_ema < 0.f) ? median : (0.5f * median + 0.5f * scan_ema);
          scan_mm   = scan_ema;
          valid_count = bc;
        }
      }

      float out_vy  = 0.f;
      bool  done_vy = false;
      if (valid_count > 0) {
        // kiri: err = target - scan (mendekati kiri → +vy)
        // kanan: err = scan - target (menjauhi kanan → +vy, arah terbalik)
        float err_vy = use_kanan ? (scan_mm - target_scan) : (target_scan - scan_mm);
        out_vy  = clamp(kScanKpVy * err_vy, kPlaceMaxVy);
        done_vy = std::fabs(err_vy) < kScanTolVy;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
        "  PLACE_BOX g%d: scan_%s=%.0fmm vy=%.3f | %s=%.0fmm vx=%.3f | yaw_err=%.1f° wz=%.3f | %s%s",
        g, use_kanan ? "kanan" : "kiri", scan_mm, out_vy,
        (rak == 1 && use_g12) ? "s1" : (g == 3 && rak == 3) ? "s2" : "avg", vx_src, out_vx, err_yaw, out_wz,
        done_vx ? "VX✓ " : "VX.. ", done_vy ? "VY✓" : "VY..");

      if (done_vx && done_vy) { stop(); break; }

      cmdVel(done_vx ? 0.0 : (double)out_vx,
             done_vy ? 0.0 : (double)out_vy,
             (double)out_wz);
      delay(20);
    }

    // ── release sequence per gripper ─────────────────────────────────────
    RCLCPP_INFO(get_logger(), "  PLACE_BOX: release gripper %d", g);

    switch (g) {

      case 1:
        lifterTuret(1, -2000);
        delay(1000);
        sudutTuret(1, 120);
        delay(1000);
        sudutTuret(1, 230);
        griper(1, kGriperOpen);
        delay(1200);
        sudutTuret(1, 0);
        lifterTuret(1, -2000);
        delay(1000);
        griper(1, kGriperClose);
        delay(1000);
        lifterTuret(1, -500);
        break;

      case 3:
        lifterTuret(2, -2000);
        delay(1000);
        sudutTuret(2, 120);
        delay(1000);
        sudutTuret(2, 230);
        griper(3, kGriperOpen);
        delay(1200);
        sudutTuret(2, 0);
        lifterTuret(2, -2000);
        delay(1000);
        griper(3, kGriperClose);
        delay(1000);
        lifterTuret(2, -500);
        break;

      case 4:
        lifterTuret(1, -2000);
        delay(1000);
        sudutTuret(1, -120);
        delay(1000);
        sudutTuret(1, -230);
        griper(2, kGriperOpen);
        delay(1200);
        sudutTuret(1, 0);
        lifterTuret(1, -2000);
        delay(1000);
        griper(2, kGriperClose);
        delay(1000);
        lifterTuret(1, -500);
        break;

      case 2:
        lifterTuret(2, -2000);
        delay(1000);
        sudutTuret(2, -120);
        delay(1000);
        sudutTuret(2, -230);
        griper(4, kGriperOpen);
        delay(1200);
        sudutTuret(2, 0);
        lifterTuret(2, -2000);
        delay(1000);
        griper(4, kGriperClose);
        delay(1000);
        lifterTuret(2, -500);
        break;
    }

    gripper_used_[g - 1] = false;  // tandai kosong setelah release
    tictactoe_place_idx_++;        // hitung jumlah placement (untuk skip mundur pertama)
  }

  // release-only untuk fase 2 (7-9): robot sudah diposisikan manual, skip semua positioning
  void stepPlaceBoxRelease()
  {
    static constexpr int kPlaceOrder[4] = {1, 3, 4, 2};
    int g = -1;
    for (int i = 0; i < 4; i++) {
      if (gripper_used_[kPlaceOrder[i] - 1]) { g = kPlaceOrder[i]; break; }
    }
    if (g == -1) {
      RCLCPP_WARN(get_logger(), "  PLACE_RELEASE: tidak ada gripper terisi!");
      return;
    }
    RCLCPP_INFO(get_logger(), "=== PLACE_RELEASE: gripper=%d ===", g);

    // lifterHw(-650);
    // delay(1000);

    lifterHw(-700);

    switch (g) {
      case 1:
        lifterTuret(1, -2000);
        delay(1000);
        sudutTuret(1, 120);
        delay(1000);
        sudutTuret(1, 250);
        griper(1, kGriperOpen);
        delay(3000);
        sudutTuret(1, 0);
        lifterTuret(1, -2000);
        delay(1000);
        griper(1, kGriperClose);
        delay(1000);
        lifterTuret(1, -500);
        break;

      case 3:
        lifterTuret(2, -2000);
        delay(1000);
        sudutTuret(2, 120);
        delay(1000);
        sudutTuret(2, 250);
        griper(3, kGriperOpen);
        delay(3000);
        sudutTuret(2, 0);
        lifterTuret(2, -2000);
        delay(1000);
        griper(3, kGriperClose);
        delay(1000);
        lifterTuret(2, -500);
        break;

      case 4:
        lifterTuret(1, -2000);
        delay(1000);
        sudutTuret(1, -120);
        delay(1000);
        sudutTuret(1, -250);
        griper(2, kGriperOpen);
        delay(3000);
        sudutTuret(1, 0);
        lifterTuret(1, -2000);
        delay(1000);
        griper(2, kGriperClose);
        delay(1000);
        lifterTuret(1, -500);
        break;

      case 2:
        lifterTuret(2, -2000);
        delay(1000);
        sudutTuret(2, -120);
        delay(1000);
        sudutTuret(2, -250);
        griper(4, kGriperOpen);
        delay(3000);
        sudutTuret(2, 0);
        lifterTuret(2, -2000);
        delay(1000);
        griper(4, kGriperClose);
        delay(1000);
        lifterTuret(2, -500);
        break;
    }

    gripper_used_[g - 1] = false;
    tictactoe_place_idx_++;
  }

  // mundur sebelum ROTATE: wall[0]==1 → mundur sampai avg tof_s1/s2 > 300mm, semua 0 → 1 detik
  void stepMundur(const ForestPlanStep & step)
  {
    bool wall0 = !step.wall.empty() && step.wall[0] == 1;
    bool all_zero = true;
    for (auto w : step.wall) if (w != 0) { all_zero = false; break; }

    float yaw_ref = forestYaw();

    if (wall0) {
      RCLCPP_INFO(get_logger(), "  mundur: wall[0]=1 → sampai tof depan > 300mm");
      while (rclcpp::ok() && running_ && !e_stop_) {
        float avg = ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
        if (tof_s1_ > 0 && tof_s2_ > 0 && avg > 300.f) { stop(); break; }
        float err_yaw = forestYaw() - yaw_ref;
        if (err_yaw >  180.f) err_yaw -= 360.f;
        if (err_yaw < -180.f) err_yaw += 360.f;
        cmdVel(-kMoveSpeed, 0.0, clamp(kMoveKpWz * err_yaw, kMoveMaxWz));
        delay(20);
      }
    } else {
      // wall[0]!=1 atau semua 0 atau kosong → mundur 500ms default
      RCLCPP_INFO(get_logger(), "  mundur: default 500ms (wall[0]=%d all_zero=%d)",
        step.wall.empty() ? -1 : step.wall[0], (int)all_zero);
      float err_yaw = forestYaw() - yaw_ref;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      cmdVel(-kMoveSpeed, 0.0, clamp(kMoveKpWz * err_yaw, kMoveMaxWz));
      delay(500);
      stop();
    }
  }

  // ACTION_ROTATE (4): putar ke heading absolut (deg) pakai IMU yaw
  void stepRotate(int heading_deg)
  {
    float target = (float)heading_deg;
    while (target >  180.f) target -= 360.f;
    while (target < -180.f) target += 360.f;

    while (rclcpp::ok() && running_ && !e_stop_) {
      float err_yaw = forestYaw() - target;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      if (std::fabs(err_yaw) < kRotateTol) { stop(); break; }

     // float out_wz = clamp(kRotateKpWz * err_yaw, kRotateMaxWz);
      float out_wz = clamp(0.02f * err_yaw, 0.3f);
      
      cmdVel(0.0, 0.0, (double)out_wz);
      delay(20);
    }
  }

  // ACTION_ENTER (2): lifter sesuai delta → maju dengan yaw hold →
  //   mid1<100 → lifter1=-10 → mid3<100 → lifter2=-10 → mid4<100 → stop
  void stepEnter(const ForestPlanStep & step)
  {
    applyElev(step);  // kirim lifter sesuai delta (200→-500, 400→-1000)

    float yaw_ref = forestYaw();
    bool  lft1_sent = false;
    bool  lft2_sent = false;

    RCLCPP_INFO(get_logger(), "  ENTER: maju yaw_ref=%.1f°", yaw_ref);

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto ms = [&]{ return std::chrono::duration_cast<std::chrono::milliseconds>(
                     clock::now() - t0).count(); };

    while (rclcpp::ok() && running_ && !e_stop_ && ms() < 10000) {
      float err_yaw = forestYaw() - yaw_ref;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      float out_wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);

      if (!lft1_sent && tof_mid1_ > 0 && tof_mid1_ < 100) {
        stop();
        lifterRobot(1, -20);
        lft1_sent = true;
        RCLCPP_INFO(get_logger(), "  ENTER: mid1<%u → lifterRobot(1,-10) delay 1s", tof_mid1_);
        delay(1000);
      }
      if (!lft2_sent && tof_mid3_ > 0 && tof_mid3_ < 100) {
        stop();
        lifterRobot(2, -20);
        lft2_sent = true;
        RCLCPP_INFO(get_logger(), "  ENTER: mid3<%u → lifterRobot(2,-10) delay 1s", tof_mid3_);
        delay(1000);
      }
      if (tof_mid4_ > 0 && tof_mid4_ < 100) {
        stop();
        lifterRobot(1, -50);
        lifterRobot(2, -50);
        RCLCPP_INFO(get_logger(), "  ENTER: mid4<%u → STOP idle", tof_mid4_);
        break;
      }

      cmdVel(0.175, 0.0, (double)out_wz);
      delay(20);
    }
  }

  // ACTION_MOVE / ENTER / EXIT: gerak open-loop (direction+heading hold)
  // dibatasi kMoveMaxMs karena delta dari planner adalah tinggi lifter (mm),
  // bukan jarak tempuh — lihat catatan applyElev().
  void stepMove(const ForestPlanStep & step)
  {
    float vx, vy;
    dirToVel(step.direction, kMoveSpeed, vx, vy);
    float yaw_ref = (float)step.heading;

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto ms = [&]{ return std::chrono::duration_cast<std::chrono::milliseconds>(
                     clock::now() - t0).count(); };

    while (rclcpp::ok() && running_ && !e_stop_ && ms() < kMoveMaxMs) {
      float err_yaw = forestYaw() - yaw_ref;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      float out_wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
      cmdVel((double)vx, (double)vy, (double)out_wz);
      delay(20);
    }
    stop();
  }

  // MOVE elev=UP: seperti stepEnter tapi mid1 → lifterRobot(1,-10) → maju → mid4 → stop
  void stepMoveUp(const ForestPlanStep & step)
  {
    float yaw_ref = forestYaw();
    bool  lft1_sent = false;

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto ms = [&]{ return std::chrono::duration_cast<std::chrono::milliseconds>(
                     clock::now() - t0).count(); };

    bool lft2_sent = false;

    while (rclcpp::ok() && running_ && !e_stop_ && ms() < 10000) {
      float err_yaw = forestYaw() - yaw_ref;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      float out_wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);

      if (!lft1_sent && tof_mid1_ > 0 && tof_mid1_ < 100) {
        stop();
        RCLCPP_INFO(get_logger(), "  MOVE UP: mid1=%d < 100 → lifterRobot(1,-10)", tof_mid1_);
        lifterRobot(1, -20);
        lft1_sent = true;
        delay(800);
        if (step.heading == 0) yaw_ref = forestYaw();
      }

      if (lft1_sent && !lft2_sent && tof_mid3_ > 0 && tof_mid3_ < 100) {
        stop();
        RCLCPP_INFO(get_logger(), "  MOVE UP: mid3=%d < 100 → lifterRobot(2,-10)", tof_mid3_);
        lifterRobot(2, -20);
        lft2_sent = true;
        delay(800);
      }

      if (lft2_sent && tof_mid4_ > 0 && tof_mid4_ < 100) {
        RCLCPP_INFO(get_logger(), "  MOVE UP: mid4=%d < 100 → stop", tof_mid4_);
        stop();
        break;
      }

      cmdVel(kMoveSpeed, 0.0, (double)out_wz);
      delay(20);
    }
    stop();
  }

  // MOVE elev=DOWN: maju sambil turunkan lifter turet per KFS yang dilewati.
  //   mid2 > 100 → stop → lifterRobot(1, -500) → delay 1s → lifterHw(-1000) → lanjut
  //   mid4 > 100 → stop → lifterRobot(2, -500) → delay 1s → selesai
  void stepMoveDown(const ForestPlanStep & step)
  {
    lifterHw(-10);
    delay(1000);
    float yaw_ref = forestYaw();
    bool  lft1_done = false;
    bool  lft2_done = false;

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto ms = [&]{ return std::chrono::duration_cast<std::chrono::milliseconds>(
                     clock::now() - t0).count(); };

    while (rclcpp::ok() && running_ && !e_stop_ && ms() < 15000) {
      float err_yaw = forestYaw() - yaw_ref;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      float out_wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);

      if (!lft1_done && tof_mid2_ > 100) {
        stop();
        RCLCPP_INFO(get_logger(), "  MOVE DOWN: mid2=%d > 100 → lifterRobot(1,-500)", tof_mid2_);
        lifterRobot(1, kElevDelta200);   // -500
        delay(800);
        lifterBaseBelakang(1000);
        lft1_done = true;
      }

      if (lft1_done && !lft2_done && tof_mid4_ > 100) {
        stop();
        RCLCPP_INFO(get_logger(), "  MOVE DOWN: mid4=%d > 100 → lifterRobot(2,-500)", tof_mid4_);
        lifterRobot(2, kElevDelta200);   // -500
        delay(800);
        lifterBaseBelakang(0);
        cmdVel(kMoveDownSpeed, 0.0, 0.0);
        delay(500);
        lft2_done = true;
        break;
      }

      cmdVel(kMoveDownSpeed, 0.0, (double)out_wz);
      delay(20);
    }
    stop();
  }

  // EXIT elev=DOWN: delta=200 → lifterRobot -500, delta=400 → lifterRobot -1000
  void stepExitDown(const ForestPlanStep & step)
  {
    lifterHw(-20);
    delay(1000);
    int16_t lft_val = (step.delta == 400) ? kElevDelta400 : kElevDelta200;
    float yaw_ref = forestYaw();
    bool  lft1_done = false;
    bool  lft2_done = false;

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto ms = [&]{ return std::chrono::duration_cast<std::chrono::milliseconds>(
                     clock::now() - t0).count(); };

    while (rclcpp::ok() && running_ && !e_stop_ && ms() < 15000) {
      float err_yaw = forestYaw() - yaw_ref;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      float out_wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);

      if (!lft1_done && tof_mid2_ > 100) {
        stop();
        RCLCPP_INFO(get_logger(), "  EXIT DOWN: mid2=%d > 100 → lifterRobot(1,%d)", tof_mid2_, lft_val);
        lifterRobot(1, lft_val);
        lifterBaseBelakang(1000);
        delay(1000);
        lft1_done = true;
      }

      if (lft1_done && !lft2_done && tof_mid4_ > 100) {
        stop();
        RCLCPP_INFO(get_logger(), "  EXIT DOWN: mid4=%d > 100 → lifterRobot(2,%d)", tof_mid4_, lft_val);
        lifterRobot(2, lft_val);
        delay(1000);
        lifterBaseBelakang(0);
        cmdVel(kMoveDownSpeed, 0.0, (double)out_wz);
        delay(800);
        lifterHw(-200);
        lft2_done = true;
        break;
      }

      cmdVel(kMoveDownSpeed, 0.0, (double)out_wz);
      delay(20);
    }
    stop();
  }

  // step.elev ("UP"/"DOWN"/"FLAT") + step.delta (mm) dari planner mengacu ke
  // tinggi lifter body, BUKAN jarak tempuh. Mapping mm → posisi lifterHw()
  // belum dikalibrasi di lapangan, jadi sengaja tidak dieksekusi otomatis
  // (menghindari salah gerak lifter). Isi TODO ini setelah kalibrasi.
  void applyElev(const ForestPlanStep & step)
  {
    if (step.elev.empty() || step.elev == "-" || step.elev == "OUTSIDE") return;
    int16_t pos = 0;
    if      (step.delta == 200) pos = kElevDelta200;
    else if (step.delta == 400) pos = kElevDelta400;
    else return;
    RCLCPP_INFO(get_logger(), "  applyElev elev=%s delta=%d → lifter %d", step.elev.c_str(), step.delta, pos);
    lifterHw(pos);
    if(pos == kElevDelta200) {delay(500);}
    if(pos == kElevDelta400) {delay(1000);}
  }

  // dispatch satu ForestPlanStep ke aksi hardware nyata
  // (struktur dispatch mengacu ke kfs_plan_executor_dummy_node.cpp)
  void executeForestStep(const ForestPlanStep & step, bool skip_reposisi = false)
  {
    RCLCPP_INFO(get_logger(),
      "[Forest] step=%d action=%d(%s) node=%d target=%d heading=%d dir=%s elev=%s delta=%d%s",
      step.step, step.action, step.action_name.c_str(), step.node, step.target,
      step.heading, step.direction.c_str(), step.elev.c_str(), step.delta,
      skip_reposisi ? " [skip_reposisi]" : "");

    switch (step.action) {
      case ForestPlanStep::ACTION_APPROACH: {
        // 1. angkat ke -1000 dulu (aman sebelum gerak lateral)
        RCLCPP_INFO(get_logger(), "  APPROACH target=%d → lifter aman -1000", step.target);
        // lifterHw(-1000);
        // delay(500);
        // 2. posisi lateral (vy selesai di sini)
        posisiForest(step.target);
        // 3. setelah vy selesai, kirim posisi lifter final untuk target ini
        int16_t lft = (step.target >= 1 && step.target <= 3) ? kApproachLifter[step.target] : -1000;
        RCLCPP_INFO(get_logger(), "  APPROACH target=%d → lifter final %d", step.target, lft);
        // lifterHw(lft);
        // delay(500);
        break;
      }

      case ForestPlanStep::ACTION_ENTER:
      //  reposisiStep(step);
        stepEnter(step);
        // if (step.target == 2) reposisiF2();
        // if (step.target == 1) reposisiF1();
        // reposisiStep(step);
        break;

      case ForestPlanStep::ACTION_MOVE:
        if (!skip_reposisi) reposisiStep(step);
        if (step.elev == "DOWN") {
          stepMoveDown(step);
        } else if (step.elev == "UP") {
          applyElev(step);
          delay(300);
          stepMoveUp(step);
        } else {
          applyElev(step);
          stepMove(step);
        }
        
        break;

      case ForestPlanStep::ACTION_EXIT:
      reposisiStep(step);
        if (step.elev == "DOWN") {
          stepExitDown(step);
        } else {
          applyElev(step);
          stepMove(step);
        }
        // reposisiStep(step);
        break;

      case ForestPlanStep::ACTION_ROTATE:
        stepRotate(step.heading);
        break;

      case ForestPlanStep::ACTION_PICK_UP:
        reposisiStep(step);
        stepPickUp(step);
        // reposisiStep(step);
        break;

      case ForestPlanStep::ACTION_VALIDATE:
        RCLCPP_WARN(get_logger(), "  -> VALIDATE: legacy, dilewati.");
        break;

      default:
        RCLCPP_WARN(get_logger(), "  -> action tidak dikenal (%d), dilewati.", step.action);
        break;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  FOREST SEQUENCE: jalankan seluruh plan_steps_ dari /intelligent/kfs_retrieval_plan
  // ════════════════════════════════════════════════════════════════════════
  void seqForest()
  {
    if (plan_steps_.empty()) {
      RCLCPP_WARN(get_logger(), "seqForest: plan KFS belum diterima, dilewati.");
      return;
    }

    forest_yaw_offset_ = imu_yaw_;
    RCLCPP_INFO(get_logger(), "seqForest: yaw offset=%.1f° (dianggap 0)", forest_yaw_offset_);

    // maju 1 detik sebelum mulai eksekusi step
    lifterTuret(1, -1000);
    sudutTuret(1, 630);
    delay(500);
    lifterHw(-500);
    delay(200);
    cmdVel(0.5f, 0.f, 0.f);
    delay(500);
    // stop();

    // snapshot lokal agar callback publish ulang tidak ganggu eksekusi
    std::vector<ForestPlanStep> steps = plan_steps_;
    int idx = 0;
    while (rclcpp::ok() && running_ && !e_stop_
           && idx < (int)steps.size()) {
      // skip reposisi jika step sebelumnya PICK_UP dengan heading sama
      // atau jika wall hanya kiri/kanan (tidak ada front wall)
      bool skip_rep = false;
      if (idx > 0 && steps[idx].action == ForestPlanStep::ACTION_MOVE) {
        if (steps[idx - 1].action == ForestPlanStep::ACTION_PICK_UP
            && steps[idx - 1].heading == steps[idx].heading) {
          skip_rep = true;
          RCLCPP_INFO(get_logger(), "  MOVE: prev=PICK_UP heading sama (%d), skip reposisi",
            steps[idx].heading);
        } else if (steps[idx].wall.empty() || steps[idx].wall[0] == 0) {
          skip_rep = true;
          RCLCPP_INFO(get_logger(), "  MOVE: no front wall, skip reposisi");
        }
      }
      executeForestStep(steps[idx], skip_rep);

      // setelah PICK_UP: mundur jika next step heading berbeda
      if (steps[idx].action == ForestPlanStep::ACTION_PICK_UP) {
        int next = idx + 1;
        if (next < (int)steps.size() && steps[next].heading != steps[idx].heading) {
          RCLCPP_INFO(get_logger(), "  PICK_UP done: heading %d→%d beda, mundur dulu",
            steps[idx].heading, steps[next].heading);
          stepMundur(steps[idx]);
        }
      }

      idx++;
    }
    stop();
  }

  // ════════════════════════════════════════════════════════════════════════
  //  AFTER FOREST: phase 1 — maju avg(s1,s2) target 400mm + wz dari s2-s1
  // ════════════════════════════════════════════════════════════════════════
  void seqForestPos1()
  {
    RCLCPP_INFO(get_logger(), "=== seqForestPos1: maju target 400mm ===");
    while (rclcpp::ok() && running_ && !e_stop_) {
      if (tof_s1_ == 0 || tof_s2_ == 0) { delay(20); continue; }

      float avg    = ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
      float err_vx = avg - 400.f;
      float out_vx = clamp(kApproachKpVx * err_vx, kApproachMaxVx);

      // wz: paralel dinding dari selisih s2-s1
      float diff   = (float)tof_s2_ - (float)tof_s1_;
      float out_wz = clamp(kApproachKpWz * diff, kApproachMaxWz);

      if (std::fabs(err_vx) < kApproachTolVx) { stop(); break; }

      cmdVel((double)out_vx, 0.0, (double)out_wz);
      delay(20);
    }
    RCLCPP_INFO(get_logger(), "=== seqForestPos1: selesai ===");
  }

  // ════════════════════════════════════════════════════════════════════════
  //  AFTER FOREST: phase 2 — vy dari scan kiri (filter, mm, target 700mm)
  //                           vx berda00mm, nonaktif (0) jika >= 1200mm
  // ════════════════════════════════════════════════════════════════════════
  void seqForestPos2()
  {
    bool  is_merah = (gui_team_ == 1);
    RCLCPP_INFO(get_logger(), "=== seqForestPos2: vy scan %s + vx s1 ===",
      is_merah ? "kiri" : "kanan");
    float yaw_ref = forestYaw();

    while (rclcpp::ok() && running_ && !e_stop_) {
      int   n           = (int)scan_.ranges.size();
      float scan_mm     = 0.f;
      int   valid_count = 0;
      if (n > 0) {
        if (is_merah) {
          // merah: scan kiri (idx n-6 .. n-1)
          int i_center = n - 1;
          for (int i = i_center - 5; i <= i_center; i++) {
            if (i < 0 || i >= n) continue;
            float r = scan_.ranges[i];
            if (std::isfinite(r) && r >= scan_.range_min && r <= scan_.range_max)
              { scan_mm += r * 1000.f; valid_count++; }
          }
        } else {
          // biru: scan kanan (idx 0-5)
          for (int i = 0; i <= 5 && i < n; i++) {
            float r = scan_.ranges[i];
            if (std::isfinite(r) && r >= scan_.range_min && r <= scan_.range_max)
              { scan_mm += r * 1000.f; valid_count++; }
          }
        }
        if (valid_count > 0) scan_mm /= (float)valid_count;
      }

      // vy: merah → 700 - scan (mendekati kiri), biru → scan - 700 (mendekati kanan)
      float out_vy  = 0.f;
      bool  done_vy = false;
      if (valid_count > 0) {
        float err_vy = is_merah ? (700.f - scan_mm) : (scan_mm - 700.f);
        out_vy  = clamp(kScanKpVy * err_vy, kScanMaxVy);
        done_vy = std::fabs(err_vy) < kScanTolVy;
      }
      if (done_vy) { stop(); break; }

      // vx: s1 target 400mm, nonaktif jika scan < 1200mm
      float out_vx = 0.f;
      if (tof_s1_ > 0 && !(valid_count > 0 && scan_mm < 1200.f)) {
        float err_vx = (float)tof_s1_ - 400.f;
        out_vx = clamp(kApproachKpVx * err_vx, kApproachMaxVx);
      }

      // wz: IMU yaw hold
      float err_yaw = forestYaw() - yaw_ref;
      if (err_yaw >  180.f) err_yaw -= 360.f;
      if (err_yaw < -180.f) err_yaw += 360.f;
      float out_wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "  pos2: scan_%s=%.0fmm vy=%.3f | s1=%umm vx=%.3f | yaw_err=%.1f° wz=%.3f",
        is_merah ? "kanan" : "kiri", scan_mm, out_vy, tof_s1_, out_vx, err_yaw, out_wz);

      cmdVel((double)out_vx, (double)out_vy, (double)out_wz);
      delay(20);
    }
    stop();
    RCLCPP_INFO(get_logger(), "=== seqForestPos2: selesai ===");
  }

  // ════════════════════════════════════════════════════════════════════════
  //  TICTACTOE SEQUENCE
  //
  //  Layout papan (dilihat dari depan):
  //    9 | 8 | 7      kolom:  7,4 → rak 1 (700mm)
  //    6 | 5 | 4              8,5 → rak 2 (1200mm)
  //    3 | 2 | 1              9,6 → rak 3 (1700mm)
  //
  //  Robot hanya bisa menjangkau posisi 4–9.
  //  Posisi 4 dan 7 share kolom yang sama (scan kiri sama), begitu seterusnya.
  //  Input: gui_tictactoe_ (array dari GUI, urutan eksekusi dari index 0)
  //  Mapping pos → rak: pos%3==1 → rak1, pos%3==2 → rak2, pos%3==0 → rak3
  // ════════════════════════════════════════════════════════════════════════
  void seqTictactoe()
  {
    // urutan naruh: gripper 1 → gripper 3 → ROTATE 180° → gripper 2 → gripper 4
    //
    //   placed=1 → gripper 1
    //   placed=2 → gripper 3
    //   ── rotate 180° ──
    //   placed=3 → gripper 2
    //   placed=4 → gripper 4

    RCLCPP_INFO(get_logger(), "=== TICTACTOE mulai: %zu posisi ===", gui_tictactoe_.size());

    // DUMMY: anggap gripper 1 dan 3 sudah terisi box
    // gripper_used_[0] = true;  // gripper 1
    // gripper_used_[2] = true;  // gripper 3

    // pisahkan posisi 4-6 (bawah, robot jalan sendiri) dan 7-9 (atas, diangkut robot manual)
    std::vector<int32_t> pos_bawah, pos_atas;
    for (int32_t pos : gui_tictactoe_) {
      if (pos >= 4 && pos <= 6)       pos_bawah.push_back(pos);
      else if (pos >= 7 && pos <= 9)  pos_atas.push_back(pos);
    }

    // ── fase 1: taruh di posisi 4-6 ─────────────────────────────────────────
    int placed = 0;
    float fase1_yaw_ref = forestYaw();
    for (int32_t pos : pos_bawah) {
      if (!rclcpp::ok() || !running_ || e_stop_) break;
      int rak = (pos % 3 == 1) ? 1 : (pos % 3 == 2) ? 2 : 3;
      RCLCPP_INFO(get_logger(), "=== TICTACTOE bawah: pos=%d rak=%d placed=%d ===", pos, rak, placed + 1);
      stepPlaceBox(rak);
      placed++;
    }
    // mundur setelah placement terakhir di 4-6
    if (placed > 0 && rclcpp::ok() && running_ && !e_stop_) {
      RCLCPP_INFO(get_logger(), "  TICTACTOE: mundur setelah fase 1");
      float ey = forestYaw() - fase1_yaw_ref;
      if (ey >  180.f) ey -= 360.f;
      if (ey < -180.f) ey += 360.f;
      cmdVel(-kMoveSpeed, 0.0, 0.0);
      delay(1000);
      stop();
    }

    // ── fase 2: standby + tunggu tombol untuk posisi 7-9 ────────────────────
    if (!pos_atas.empty() && rclcpp::ok() && running_ && !e_stop_) {
      RCLCPP_INFO(get_logger(), "=== TICTACTOE: standby -1000, tunggu btn untuk %zu posisi atas ===",
      pos_atas.size());
      stop();
      delay(1000);
      lifterHw(-1100);
      


      // lifterTuret(1, -3000);

      for (int32_t pos : pos_atas) {
        if (!rclcpp::ok() || !running_ || e_stop_) break;

        // rotate turet utama sebelum gripper 4 (setelah gripper 1 dan 3 di-place)
        {
          static constexpr int kPlaceOrder[4] = {1, 3, 4, 2};
          int next_g = -1;
          for (int i = 0; i < 4; i++) {
            if (gripper_used_[kPlaceOrder[i] - 1]) { next_g = kPlaceOrder[i]; break; }
          }
          if (next_g == 4) {
            RCLCPP_INFO(get_logger(), "  TICTACTOE: rotate turet utama untuk gripper 4");
            turetUtama(770);
            delay(2500);
          }
        }

        // tunggu btn_tictactoe[0] != 0
        RCLCPP_INFO(get_logger(), "  TICTACTOE atas: pos=%d, tunggu btn_tictactoe[0]...", pos);
        web_btn_tictactoe0_ = 0;
        while (rclcpp::ok() && running_ && !e_stop_) {
          if (web_btn_tictactoe0_ != 0) break;
          delay(50);
        }
        if (!rclcpp::ok() || !running_ || e_stop_) break;

        web_btn_tictactoe0_ = 0;  // reset setelah diterima
        RCLCPP_INFO(get_logger(), "=== TICTACTOE atas: pos=%d placed=%d ===", pos, placed + 1);
        stepPlaceBoxRelease();
        placed++;
      }
    }

    stop();
    RCLCPP_INFO(get_logger(), "=== TICTACTOE: selesai (%d posisi dieksekusi) ===", placed);
  }

  // ════════════════════════════════════════════════════════════════════════
  //  LOOP UTAMA
  // ════════════════════════════════════════════════════════════════════════
  void loop()
  {
    while (rclcpp::ok() && running_) {
      if (!rclcpp::ok() || !running_) break;
      if (e_stop_) continue;

      // ── seq normal (dinonaktifkan sementara) ─────────────────────────
      waitForStart();

      // // ── TEST PICK UP: tekan joystick btn[0] setelah start ─────────────
      // if (joy_pickup_req_) {
      //   joy_pickup_req_ = false;
      //   ForestPlanStep test_step;
      //   test_step.elev  = "UP";
      //   test_step.delta = 0;
      //   RCLCPP_INFO(get_logger(), "=== TEST PICK UP (btn[0]) ===");
      //   stepPickUp(test_step);
      //   waitForRelease();
      // }

      if(!gui_retry_.empty() && gui_retry_[0] == 1) 
      { 
        seqSpearhead();
      }

      if(!gui_retry_.empty() && gui_retry_[1] == 1) 
      {
            RCLCPP_INFO(get_logger(), "=== SEQ: forest mulai ===");
            seqForest();
            RCLCPP_INFO(get_logger(), "=== SEQ: forest selesai ===");
            seqForestPos1();
            seqForestPos2();
              // maju 3s
              {
                float yaw_ref = forestYaw();
                using clock = std::chrono::steady_clock;
                auto t0 = clock::now();
                while (rclcpp::ok() && running_ && !e_stop_) {
                  if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        clock::now() - t0).count() >= 3000) { stop(); break; }
                  float err_yaw = forestYaw() - yaw_ref;
                  if (err_yaw >  180.f) err_yaw -= 360.f;
                  if (err_yaw < -180.f) err_yaw += 360.f;
                  cmdVel(0.5, 0.0, (double)clamp(kMoveKpWz * err_yaw, kMoveMaxWz));
                  delay(20);
                }
              }
              // pos 300mm
              {
                float yaw_ref = forestYaw();
                while (rclcpp::ok() && running_ && !e_stop_) {
                  if (tof_s1_ == 0 || tof_s2_ == 0) { delay(20); continue; }
                  float avg = ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
                  float err_vx = avg - 300.f;
                  float out_vx = clamp(kApproachKpVx * err_vx, kApproachMaxVx);
                  float err_yaw = forestYaw() - yaw_ref;
                  if (err_yaw >  180.f) err_yaw -= 360.f;
                  if (err_yaw < -180.f) err_yaw += 360.f;
                  if (std::fabs(err_vx) < kApproachTolVx) { stop(); break; }
                  cmdVel((double)out_vx, 0.0, (double)clamp(kMoveKpWz * err_yaw, kMoveMaxWz));
                  delay(20);
                }
              }

      
            //benerin box lalu retry
            delay(1000);
            turetUtama(0);
            waitForStart();


            // scan 1500mm + vx avg(s1,s2) → 600mm  (biru: scan kiri, merah: scan kanan)
            {
              bool  is_merah_pos = (gui_team_ == 1);
              float yaw_ref = forestYaw();
              auto  t0      = std::chrono::steady_clock::now();
              while (rclcpp::ok() && running_ && !e_stop_) {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();
                if (elapsed_ms > 10000) { stop(); break; }

                // vx: avg tof_s1 dan tof_s2 → target 600mm
                bool  done_vx = true;
                float out_vx  = 0.f;
                if (tof_s1_ > 0 && tof_s2_ > 0) {
                  float avg_s  = ((float)tof_s1_ + (float)tof_s2_) * 0.5f;
                  float err_vx = avg_s - 600.f;
                  out_vx  = clamp(kApproachKpVx * err_vx, kApproachMaxVx);
                  done_vx = std::fabs(err_vx) < kApproachTolVx;
                }

                // vy: merah → scan kanan (idx 0-5), biru → scan kiri (idx n-6 .. n-1)
                int   n = (int)scan_.ranges.size();
                float scan_mm = 0.f; int vc = 0;
                if (is_merah_pos) {
                  for (int i = 0; i <= 5 && i < n; i++) {
                    float r = scan_.ranges[i];
                    if (std::isfinite(r) && r >= scan_.range_min && r <= scan_.range_max)
                      { scan_mm += r * 1000.f; vc++; }
                  }
                } else {
                  for (int i = n - 6; i < n; i++) {
                    if (i < 0) continue;
                    float r = scan_.ranges[i];
                    if (std::isfinite(r) && r >= scan_.range_min && r <= scan_.range_max)
                      { scan_mm += r * 1000.f; vc++; }
                  }
                }
                if (vc == 0) { delay(20); continue; }
                scan_mm /= (float)vc;
                float err_vy = is_merah_pos ? (scan_mm - 1500.f) : (1500.f - scan_mm);
                bool  done_vy = std::fabs(err_vy) < kScanTolVy;

                float err_yaw = forestYaw() - yaw_ref;
                if (err_yaw >  180.f) err_yaw -= 360.f;
                if (err_yaw < -180.f) err_yaw += 360.f;

                if (done_vx && done_vy) { stop(); break; }
                cmdVel(done_vx ? 0.0 : (double)out_vx,
                      done_vy ? 0.0 : (double)clamp(kScanKpVy * err_vy, kScanMaxVy),
                      (double)clamp(kMoveKpWz * err_yaw, kMoveMaxWz));
                delay(20);
              }
            }
            // rotate: merah -90°, biru +90°
            {
              bool  is_merah_pos = (gui_team_ == 1);
              float target_yaw = is_merah_pos ? (forestYaw() - 90.f) : (forestYaw() + 90.f);
              if (target_yaw >  180.f) target_yaw -= 360.f;
              if (target_yaw < -180.f) target_yaw += 360.f;
              auto  t0 = std::chrono::steady_clock::now();
              while (rclcpp::ok() && running_ && !e_stop_) {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0).count();
                if (elapsed_ms > 15000) { stop(); break; }
                float err_yaw = forestYaw() - target_yaw;
                if (err_yaw >  180.f) err_yaw -= 360.f;
                if (err_yaw < -180.f) err_yaw += 360.f;
                if (std::fabs(err_yaw) < kRotateTol) { stop(); break; }
                cmdVel(0.0, 0.0, (double)clamp(kRotateKpWz * err_yaw, kRotateMaxWz));
                delay(20);
              }
            }
            stop();

            seqTictactoe();

        }

      if(!gui_retry_.empty() && gui_retry_[2] == 1) 
      {
        for (int i = 0; i < 4; i++) gripper_used_[i] = false;
        for (int i = 0; i < 4; i++) gripper_used_[i] = false;
        gripper_used_[0] = true;  // gripper 1 dianggap sudah berisi box

        lifterTuret(1, -1000);
        sudutTuret(1, 630);
        delay(500);
        griper(1, 1);
        lifterTuret(1, -3000);
        // // delay(1000);
        // // sudutTuret(1, 720);
        // // delay(1000);
        // // griper(1, 0);
        // // delay(1000);
        // // sudutTuret(1, 0);
        // // delay(500);
        // // lifterTuret(1, -1000);
        // // delay(2000);

        lifterHw(-100);
        delay(500); 
        cmdVel(0.7, 0.0, 0.0);
        delay(1700); 
        cmdVel(0.0, 0.2, 0.0);
        delay(500); 
        stop();

        // kontrol posisi: vx dari depth kamera (target 300mm), vy dari cx frame (center 640px)
        {
          static constexpr float kCamKpVx  = 0.0008f;
          static constexpr float kCamMaxVx  = 0.25f;
          static constexpr float kCamTolVx  = 20.f;   // mm
          static constexpr float kCamKpVy  = 0.0008f;
          static constexpr float kCamMaxVy  = 0.20f;
          static constexpr float kCamTolVy  = 20.f;   // px
          static constexpr int   kFrameCx   = 640;    // center frame 1280px

          while (rclcpp::ok() && running_ && !e_stop_) {
            if (!cam_has_data_) { delay(20); continue; }

            float err_vx = cam_depth_ * 1000.f - 300.f;  // target 300mm
            float err_vy = static_cast<float>(kFrameCx - cam_x_);

            bool done_vx = std::fabs(err_vx) < kCamTolVx;
            bool done_vy = std::fabs(err_vy) < kCamTolVy;

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
              "cam ctrl | depth=%.0fmm err_vx=%.0f | cx=%d err_vy=%.0f | %s %s",
              cam_depth_ * 1000.f, err_vx, cam_x_, err_vy,
              done_vx ? "VX✓" : "VX..", done_vy ? "VY✓" : "VY..");

            if (done_vx && done_vy) { stop(); break; }

            float vx = done_vx ? 0.f : clamp(kCamKpVx * err_vx, kCamMaxVx);
            float vy = done_vy ? 0.f : clamp(kCamKpVy * err_vy, kCamMaxVy);
            cmdVel((double)vx, (double)vy, 0.0);
            delay(20);
          }
        }

        {
          float grip_yaw_ref = forestYaw();
          auto  t0           = std::chrono::steady_clock::now();
          float last_vy      = 0.05f;
          while (rclcpp::ok() && running_ && !e_stop_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms > 5000) { stop(); break; }
            int32_t s6 = tof_gripper_[6];
            int32_t s7 = tof_gripper_[7];
            if (s6 > 350 && s7 > 350) { stop(); break; }
            if      (s6 > 400) last_vy = -0.1f;
            else if (s6 < 300) last_vy =  0.1f;
            float err_yaw = forestYaw() - grip_yaw_ref;
            if (err_yaw >  180.f) err_yaw -= 360.f;
            if (err_yaw < -180.f) err_yaw += 360.f;
            float wz = clamp(kMoveKpWz * err_yaw, kMoveMaxWz);
            cmdVel(0.0, (double)last_vy, (double)wz);
            delay(20);
          }
        }

        sudutTuret(1, 820);
        delay(1000);
        griper(1, 0);
        delay(1000);
        sudutTuret(1, 0);
        delay(500);
        lifterTuret(1, -1000);
        delay(2000);

        seqTictactoe();

      }

      

    }
  }

  void nextPlanStep() { plan_step_idx_++; }

  const ForestPlanStep & currentStep() const
  {
    static ForestPlanStep empty{};
    if (plan_steps_.empty() || plan_step_idx_ >= (int)plan_steps_.size()) return empty;
    return plan_steps_[plan_step_idx_];
  }

  // ── state GUI ─────────────────────────────────────────────────────────────
  int  gui_step_{0};
  int  gui_team_{0};
  int  gui_spearhead_{0};
  int  gui_kfsret_{0};
  bool gui_ready_{false};
  bool gui_start_{false};
  std::vector<int32_t> gui_forest_, gui_tictactoe_, gui_retry_;

  // ── KFS plan ──────────────────────────────────────────────────────────────
  std::vector<ForestPlanStep> plan_steps_;
  int      plan_step_count_{0};
  int      plan_step_idx_{0};
  uint32_t plan_stamp_sec_{0};
  uint32_t plan_stamp_nanosec_{0};
  std::vector<int32_t> plan_path_nodes_;
  std::vector<int32_t> plan_picked_kfs_;
  float    plan_total_cost_{0.f};

  // ── status gripper (true = sudah terisi KFS, tidak boleh dipakai lagi) ──
  bool gripper_used_[4]{false, false, false, false};
  bool turet_rotated_{false};   // turetUtama -760 sudah diputar sekali
  int  tictactoe_place_idx_{0}; // counter urutan naruh: 0=g1, 1=g3, 2=g2, 3=g4

  // ── web gui button ────────────────────────────────────────────────────────
  int32_t web_btn_tictactoe0_{0};  // btn_tictactoe[0] dari /web_gui_data

  // ── joystick / e-stop ─────────────────────────────────────────────────────
  int  joy_test_gripper_{0};
  bool prev_joy_btn_[4]{false, false, false, false};
  bool e_stop_{false};
  bool joy_lifter_test_active_{false};
  bool joy_pickup_req_{false};
  double joy_vx_{0.0}, joy_vy_{0.0}, joy_wz_{0.0};
  bool  pos_ctrl_active_{false};
  float pos_ctrl_yaw_ref_{0.f};
  float yaw_ref_{0.f};          // heading referensi setelah reposisiF2
  float forest_yaw_offset_{0.f}; // IMU yaw saat masuk seqForest → dianggap 0

  // ── sensor ────────────────────────────────────────────────────────────────
  sensor_msgs::msg::LaserScan scan_;
  int32_t  tof_gripper_[8]{0};
  uint32_t tof_s1_{0}, tof_s2_{0}, tof_s3_{0}, tof_s4_{0};
  uint32_t tof_mid1_{0}, tof_mid1_2_{0}, tof_mid2_{0}, tof_mid3_{0}, tof_mid4_{0};
  float    wall_kanan_{-1.f}, wall_kiri_{-1.f};
  float    imu_yaw_{0.f};
  int      cam_x_{0}, cam_y_{0};
  float    cam_depth_{0.f};
  bool     cam_has_data_{false};

  bool running_{true};
  std::thread loop_thread_;

  // ── subscribers ───────────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr              step_sub_;
  rclcpp::Subscription<eira_msgs::msg::GuiData>::SharedPtr           gui_sub_;
  rclcpp::Subscription<eira_msgs::msg::KfsRetrievalPlan>::SharedPtr  plan_sub_;
  rclcpp::Subscription<sensor_tof::msg::TofArray>::SharedPtr         tof_sub_;
  rclcpp::Subscription<sensor_tof::msg::TofGripper>::SharedPtr      tof_gripper_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr             joy_sub_;
  rclcpp::Subscription<eira_msgs::msg::WebGuiData>::SharedPtr        web_gui_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            wall_kanan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            wall_kiri_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             cam_sub_;

  // ── publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       lifter_robot1_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       lifter_robot2_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       dynamixel1_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       penumatic_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       turet_utama_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       turet1_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       turet2_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       lifter_turet1_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       lifter_turet2_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       griper1_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       griper2_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       griper3_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       griper4_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       lifter_base_belakang_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeqNode>());
  rclcpp::shutdown();

  return 0;
}
