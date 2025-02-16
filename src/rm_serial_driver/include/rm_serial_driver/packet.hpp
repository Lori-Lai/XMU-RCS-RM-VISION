// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum = 0;
} __attribute__((packed));

struct NavPacket
{
    uint8_t header = 0xA6;  // 数据包的头部
    struct
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
    } game_robot_HP_t;  // 所有机器人血量数据

    struct 
    {
        uint32_t event_data;
    }event_data_t;  // 场地事件数据

    uint16_t current_HP;  // 机器人当前血量数据

    struct
    {
        uint8_t game_type;        // 比赛类型
        uint8_t game_progress;    // 当前比赛阶段
        uint16_t stage_remain_time;   // 当前阶段剩余时间
    } game_status_t;  // 比赛状态数据

    uint16_t projectile_allowance_t;  // 剩余子弹数量

    uint32_t sentry_info;  // 哨兵自主决策信息

    struct
    {
        float target_position_x;
        float target_position_y;
        uint8_t cmd_keyboard;
        uint8_t target_robot_id;
        uint8_t cmd_source;
    } map_command_t;  // 选手端下发的数据

    uint16_t checksum = 0;  // 校验和
}__attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  // bool tracking : 1;
  // uint8_t id : 3;          // 0-outpost 6-guard 7-base
  // uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  // uint8_t reserved : 1;
  // float x;
  // float y;
  // float z;
  // float yaw;
  // float vx;
  // float vy;
  // float vz;
  // float v_yaw;
  // float r1;
  // float r2;
  // float dz;
  float pitch;
  float yaw;
  float yaw_diff;
  float pitch_diff;
  float distance;
  bool fire_advice;
  uint16_t checksum = 0;
} __attribute__((packed));

struct CmdVelPacket
{
  uint8_t header = 0x6A;
  float linear_x;
  float linear_y;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline NavPacket nav_fromVector(const std::vector<uint8_t> & data)
{
  NavPacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

inline std::vector<uint8_t> cmd_toVector(const CmdVelPacket & data)
{
  std::vector<uint8_t> packet(sizeof(CmdVelPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(CmdVelPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
