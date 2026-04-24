#include "hand_position_controller/hand_position_controller.hpp"

#include <stdexcept>

namespace hand_position_controller
{

// ── 기본 프리셋 (tick 단위, center = 2048) ──────────────────────────────
// 실제 로봇에 맞게 수정하거나 YAML 파라미터로 덮어쓸 수 있음
static const std::map<std::string, std::array<int32_t, HAND_DOF>> DEFAULT_PRESETS = {
    {"open",  {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048}},
    {"close", {1200, 1800, 2400, 1200, 1800, 2400, 2800, 1200, 1800, 2400, 2800, 2048}},
    {"pinch", {2048, 2200, 2500, 2048, 2200, 2500, 2048, 1800, 1600, 1800, 2048, 2048}},
};

HandPositionController::HandPositionController(const rclcpp::NodeOptions & options)
: Node("hand_position_controller", options)
{
    declare_parameter<std::string>("port",    "/dev/ttyUSB0");
    declare_parameter<int>        ("baudrate", 1000000);

    const std::string port     = get_parameter("port").as_string();
    const int         baudrate = get_parameter("baudrate").as_int();

    // SDK 초기화
    gripper_ = std::make_unique<gripper::Gripper>(port, baudrate);
    gripper_->disable();
    gripper_->setMode(gripper::MODE_POSITION);
    gripper_->enable();

    // 프리셋 로드 (파라미터 → 없으면 기본값)
    loadPresets();

    preset_sub_ = create_subscription<std_msgs::msg::String>(
        "~/preset", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { onPreset(msg); });

    RCLCPP_INFO(get_logger(), "HandPositionController ready on %s @ %d bps", port.c_str(), baudrate);
}

HandPositionController::~HandPositionController()
{
    if (gripper_) {
        gripper_->disable();
        gripper_->close();
    }
}

// ── 프리셋 이름 수신 시 호출 ──────────────────────────────────────────
void HandPositionController::onPreset(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string & name = msg->data;
    auto it = presets_.find(name);
    if (it == presets_.end()) {
        RCLCPP_WARN(get_logger(), "Unknown preset: '%s'", name.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Sending preset: '%s'", name.c_str());
    sendPosition(it->second);
}

// ── YAML 파라미터에서 프리셋 로드 ────────────────────────────────────
// 파라미터 형식:
//   presets.my_pose: [2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048]
bool HandPositionController::loadPresets()
{
    presets_ = DEFAULT_PRESETS;

    // 파라미터 서버에서 presets.* 형태의 파라미터를 읽어 기본값을 덮어씀
    auto params = list_parameters({"presets"}, 2);
    for (const auto & name : params.names) {
        const auto ticks = get_parameter(name).as_integer_array();
        if (static_cast<int>(ticks.size()) != HAND_DOF) {
            RCLCPP_WARN(get_logger(),
                "Preset '%s' has %zu values, expected %d — skipped.",
                name.c_str(), ticks.size(), HAND_DOF);
            continue;
        }
        const std::string key = name.substr(name.rfind('.') + 1);
        std::array<int32_t, HAND_DOF> arr;
        for (int i = 0; i < HAND_DOF; ++i) {
            arr[i] = static_cast<int32_t>(ticks[i]);
        }
        presets_[key] = arr;
        RCLCPP_INFO(get_logger(), "Loaded preset: '%s'", key.c_str());
    }
    return true;
}

// ── 위치 명령 전송 ────────────────────────────────────────────────────
void HandPositionController::sendPosition(const std::array<int32_t, HAND_DOF> & positions)
{
    gripper_->writePositions(positions.data());
}

}  // namespace hand_position_controller


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hand_position_controller::HandPositionController>());
    rclcpp::shutdown();
    return 0;
}
