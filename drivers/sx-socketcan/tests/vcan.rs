//! The socket path end to end on a virtual bus, where the host has one: `vcan0` up
//! (`ip link add vcan0 type vcan && ip link set vcan0 mtu 72 up`). Without it the test says so
//! and passes, since the frame layout and error meanings are covered on every host by the
//! crate's unit tests.

#![cfg(target_os = "linux")]

use std::time::{Duration, Instant};

use sx_embodiment_drivers::can::{CanFrame, CanPort};
use sx_socketcan::SocketCanPort;

fn counter(port: &SocketCanPort, name: &str) -> u64 {
    port.counters()
        .into_iter()
        .find(|(counter, _)| *counter == name)
        .map_or(0, |(_, value)| value)
}

#[test]
fn frames_cross_a_virtual_bus_and_a_wait_sleeps_to_its_deadline() {
    if !std::path::Path::new("/sys/class/net/vcan0").exists() {
        eprintln!("no vcan0 on this host: the socket path is not exercised here");
        return;
    }
    let mut ours = SocketCanPort::open("vcan0").expect("vcan0");
    let mut other = SocketCanPort::open_virtual("vcan0").expect("vcan0, unlocked");
    assert!(SocketCanPort::open("vcan0").is_err(), "the lock holds");
    assert!(
        SocketCanPort::open_virtual("lo").is_err(),
        "only a vcan or vxcan"
    );

    let deadline = Instant::now() + Duration::from_millis(20);
    assert_eq!(ours.receive_until(deadline).expect("wait"), None);
    let late = Instant::now().saturating_duration_since(deadline);
    assert!(
        late < Duration::from_millis(5),
        "woke {late:?} after the deadline"
    );

    let burst: Vec<CanFrame> = (0..40_u8)
        .map(|n| CanFrame {
            id: 0x7FF,
            data: [n, 0, 0x33, 21, 0, 0, 0, 0],
        })
        .collect();
    for frame in &burst {
        ours.send(frame).expect("send");
    }
    let mut seen = Vec::new();
    let until = Instant::now() + Duration::from_millis(200);
    while seen.len() < burst.len() {
        match other.receive_until(until).expect("receive") {
            Some(frame) => seen.push(frame),
            None => break,
        }
    }
    assert_eq!(seen, burst, "every frame, in order");
    assert!(
        counter(&other, "rx_calls") < 40,
        "drained in batches, not one call a frame"
    );

    let answer = CanFrame {
        id: 0x29,
        data: [9, 0, 0x33, 21, 1, 2, 3, 4],
    };
    other.send(&answer).expect("answer");
    let got = ours
        .receive_until(Instant::now() + Duration::from_millis(100))
        .expect("receive");
    assert_eq!(got, Some(answer));
    assert_eq!(counter(&ours, "rx_frames"), 1);
    assert_eq!(counter(&ours, "error_frames"), 0);
}
