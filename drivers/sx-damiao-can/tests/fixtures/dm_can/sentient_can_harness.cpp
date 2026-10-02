// Drive Sentient-X/sentient_can's own Damiao encoder and decoder for mint.py's cross-check.
//
// Build against a checkout of Sentient-X/sentient_can at the commit mint.py cites (an empty
// linux/can.h stub is enough off Linux; only the packet code is linked):
//
//   c++ -std=c++17 -I<sentient_can>/include -I<stub> sentient_can_harness.cpp \
//       <sentient_can>/src/openarm/damiao_motor/dm_motor_control.cpp \
//       <sentient_can>/src/openarm/damiao_motor/dm_motor.cpp -o sentient_can_harness
//
// Input, one request per line:
//   mit <type> <send_id> <recv_id> <kp> <kd> <q> <dq> <tau>
//   enable|disable <type> <send_id> <recv_id>
//   decode <type> <send_id> <recv_id> <16 hex digits>
// Output, one line per request: the frame as `<id> <hex>`, or the decoded `q dq tau tmos trotor`
// with seventeen significant digits.

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>

using namespace openarm::damiao_motor;

static void print(const CANPacket& packet) {
    std::printf("%u ", packet.send_can_id);
    for (auto byte : packet.data) std::printf("%02x", byte);
    std::printf("\n");
}

int main() {
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string verb;
        int type;
        uint32_t send_id, recv_id;
        in >> verb >> type >> send_id >> recv_id;
        Motor motor(static_cast<MotorType>(type), send_id, recv_id);
        if (verb == "mit") {
            MITParam param{};
            in >> param.kp >> param.kd >> param.q >> param.dq >> param.tau;
            print(CanPacketEncoder::create_mit_control_command(motor, param));
        } else if (verb == "enable") {
            print(CanPacketEncoder::create_enable_command(motor));
        } else if (verb == "disable") {
            print(CanPacketEncoder::create_disable_command(motor));
        } else if (verb == "decode") {
            std::string hex;
            in >> hex;
            std::vector<uint8_t> data;
            for (size_t i = 0; i + 1 < hex.size(); i += 2)
                data.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
            auto state = CanPacketDecoder::parse_motor_state_data(motor, data);
            std::printf("%.17g %.17g %.17g %d %d\n", state.position, state.velocity, state.torque,
                        state.t_mos, state.t_rotor);
        }
    }
    return 0;
}
