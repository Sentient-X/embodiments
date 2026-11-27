//! OpenCV's `DICT_4X4_50`: each marker's 4x4 code, row-major from its top-left, the first row
//! in the most significant bits, 1 for a white cell (`aruco.generateImageMarker`'s picture).
//! OpenCV's 4x4 dictionaries share their first fifty codes, so `_100`, `_250` and `_1000`
//! read these IDs the same way.

pub const MARKER_SIZE: usize = 4;

pub const DICT_4X4_50: [u16; 50] = [
    46386, 3994, 13101, 39238, 21662, 31181, 40494, 50418, 65242, 53078, 63889, 4519, 3767, 10767,
    9393, 9790, 18021, 26112, 27742, 30383, 34443, 45099, 52437, 56706, 65095, 38001, 44260, 42324,
    8483, 13423, 17429, 22450, 40655, 61643, 2222, 2345, 6261, 1279, 3574, 7258, 5912, 10792,
    12940, 14514, 9448, 12011, 11583, 19300, 20526, 20499,
];

/// Marker `id`'s code, if the dictionary has it.
pub fn code(id: u32) -> Option<u16> {
    DICT_4X4_50.get(id as usize).copied()
}
