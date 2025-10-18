#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[allow(non_upper_case_globals)]
pub const hackrf_board_id_BOARD_ID_HACKRF_ONE: hackrf_board_id = hackrf_board_id_BOARD_ID_HACKRF1_OG;

#[allow(non_upper_case_globals)]
pub const hackrf_board_id_BOARD_ID_INVALID: hackrf_board_id = hackrf_board_id_BOARD_ID_UNDETECTED;
